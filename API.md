# Tomato-Picker-ROS2 API Reference

本文档面向需要通过 ROS 2 调用 Tomato-Picker 能力与任务接口的开发者

每个公共接口按以下顺序说明：

```text
用途
接口定义
参数
返回值 / 反馈
错误码
请求示例
使用注意
```

## 1. API 使用入口

| 使用目标 | ROS 2 接口 | 类型 |
| --- | --- | --- |
| 机械臂运动 | `/tomato_picker/motion/move_arm` | `tomato_picker_interfaces/action/MoveArm` |
| EEF 命令 | `/tomato_picker/eef/command` | `tomato_picker_interfaces/srv/CommandEef` |
| EEF 就绪状态 | `/tomato_picker/eef/ready` | `std_srvs/srv/Trigger` |
| 采摘任务 | `/tomato_picker/task/pick` | `tomato_picker_interfaces/action/PickTarget` |
| Planning Scene 开关 | `/tomato_picker/perception/set_scene_enabled` | `tomato_picker_interfaces/srv/SetSceneEnabled` |

`bringup.launch.py` 会先等待 SerialArm controllers 与 MoveGroup 就绪；`eef_mode:=damiao` 或 `mock` 时，还会等待 EEF `READY` 后再启动 Motion、Task 与 Perception 接口

接口定义位于：

```text
src/tomato_picker/interfaces/
├── action/
│   ├── MoveArm.action
│   └── PickTarget.action
├── msg/
│   └── TargetObject.msg
└── srv/
    ├── CommandEef.srv
    └── SetSceneEnabled.srv
```

---

## 2. `MoveArm` Action

Action：

```text
/tomato_picker/motion/move_arm
```

类型：

```text
tomato_picker_interfaces/action/MoveArm
```

用于将 Tomato-Picker 上层运动请求转换为 MoveIt 2 规划与执行

### 2.1. 命令类型

| `command_type` | 名称 | 作用 |
| --- | --- | --- |
| `0` | `HOME` | 移动到 MoveIt named target `home` |
| `1` | `JOINT` | 关节空间目标 |
| `2` | `POSE` | 末端位姿目标 |
| `3` | `LINE` | 笛卡尔直线路径 |

### 2.2. Goal

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `command_type` | `uint8` | `HOME / JOINT / POSE / LINE` |
| `target_pose` | `geometry_msgs/PoseStamped` | `POSE / LINE` 目标位姿 |
| `joint_names` | `string[]` | `JOINT` 可选关节名 |
| `joints` | `float64[]` | `JOINT` 目标关节位置，单位 rad |
| `velocity_scale` | `float64` | MoveIt 速度缩放 |
| `acceleration_scale` | `float64` | MoveIt 加速度缩放 |
| `execute` | `bool` | `false` 只规划，`true` 规划并执行 |

`velocity_scale <= 0` 和 `acceleration_scale <= 0` 时使用 `motion.yaml` 默认值；其他值限制到 `[0.01, 1.0]`

`JOINT` 未提供 `joint_names` 时，`joints` 数量必须与 MoveGroup 全部关节数量一致

`JOINT` 提供 `joint_names` 时，只修改给定关节，其余关节保持当前值

`POSE` 必须提供非空 `target_pose.header.frame_id`

`LINE` 的 `target_pose.header.frame_id` 必须与 MoveGroup 当前 pose reference frame 完全一致

### 2.3. Result

| 字段 | 说明 |
| --- | --- |
| `success` | 是否成功 |
| `error_code` | 错误码 |
| `message` | 结果文本 |
| `final_pose` | 结束时实际末端位姿 |
| `final_joints` | 结束时实际关节位置 |

错误码：

| 值 | 说明 |
| --- | --- |
| `0` | 成功 |
| `1` | Goal 非法或目标不受支持 |
| `2` | MoveIt 2 规划失败 |
| `3` | MoveIt 2 执行失败 |
| `4` | Action 被取消 |

### 2.4. Feedback

| 字段 | 说明 |
| --- | --- |
| `stage` | 当前阶段文本 |
| `progress` | 归一化进度 |
| `current_pose` | 当前末端位姿 |

当前阶段可能包括：

```text
SET_TARGET
CARTESIAN_PLAN
PLANNING
EXECUTING
DONE
```

### 2.5. HOME

首次真机测试建议先只规划：

```bash
ros2 action send_goal \
  /tomato_picker/motion/move_arm \
  tomato_picker_interfaces/action/MoveArm \
  "{command_type: 0, velocity_scale: 0.1, acceleration_scale: 0.1, execute: false}" \
  --feedback
```

确认安全后执行：

```bash
ros2 action send_goal \
  /tomato_picker/motion/move_arm \
  tomato_picker_interfaces/action/MoveArm \
  "{command_type: 0, velocity_scale: 0.1, acceleration_scale: 0.1, execute: true}" \
  --feedback
```

### 2.6. JOINT

指定全部六个关节：

```bash
ros2 action send_goal \
  /tomato_picker/motion/move_arm \
  tomato_picker_interfaces/action/MoveArm \
  "{command_type: 1, joint_names: [joint1, joint2, joint3, joint4, joint5, joint6], joints: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], velocity_scale: 0.1, acceleration_scale: 0.1, execute: false}" \
  --feedback
```

只修改部分关节：

```bash
ros2 action send_goal \
  /tomato_picker/motion/move_arm \
  tomato_picker_interfaces/action/MoveArm \
  "{command_type: 1, joint_names: [joint1, joint2], joints: [0.1, -0.1], velocity_scale: 0.1, acceleration_scale: 0.1, execute: false}" \
  --feedback
```

### 2.7. POSE

```bash
ros2 action send_goal \
  /tomato_picker/motion/move_arm \
  tomato_picker_interfaces/action/MoveArm \
  "{command_type: 2, target_pose: {header: {frame_id: base_link}, pose: {position: {x: 0.35, y: 0.0, z: 0.35}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}, velocity_scale: 0.1, acceleration_scale: 0.1, execute: false}" \
  --feedback
```

### 2.8. LINE

```bash
ros2 action send_goal \
  /tomato_picker/motion/move_arm \
  tomato_picker_interfaces/action/MoveArm \
  "{command_type: 3, target_pose: {header: {frame_id: base_link}, pose: {position: {x: 0.35, y: 0.0, z: 0.35}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}, velocity_scale: 0.1, acceleration_scale: 0.1, execute: false}" \
  --feedback
```

`LINE` 当前使用单个末端 waypoint，通过 `computeCartesianPath()` 生成路径；轨迹完成比例低于 `motion.yaml` 中 `cartesian_min_fraction` 时返回规划失败

### 2.9. 取消 Action

CLI 交互调用时可使用 `Ctrl+C` 结束客户端；程序调用方应使用标准 ROS 2 Action cancel 请求

Motion Server 收到取消后会调用 MoveIt `stop()` 并返回 `error_code=4`

---

## 3. `CommandEef` Service

Service：

```text
/tomato_picker/eef/command
```

类型：

```text
tomato_picker_interfaces/srv/CommandEef
```

### 3.1. 命令类型

| `command` | 名称 | `value` | 作用 |
| --- | --- | --- | --- |
| `0` | `OPEN` | 忽略 | 打开 EEF |
| `1` | `CLOSE` | 忽略 | 关闭 EEF |
| `2` | `STOP` | 忽略 | 请求停止当前 EEF 动作 |
| `3` | `SET_POSITION` | `[0, 1]` | 设置归一化位置 |

`SET_POSITION` 中：

```text
0.0 = open_position
1.0 = closed_position
```

实际电机角度由 `damiao_eef.yaml` 的 `open_position` 与 `closed_position` 映射

### 3.2. Response

| 字段 | 说明 |
| --- | --- |
| `success` | 请求是否被 Worker 接受 |
| `error_code` | 错误码 |
| `message` | 状态文本 |

错误码：

| 值 | 说明 |
| --- | --- |
| `0` | 命令已接受 |
| `1` | 不支持的命令 |
| `2` | `SET_POSITION` 超出 `[0, 1]` |
| `3` | EEF Worker 未激活 |
| `4` | STOP 尚未消费，Worker busy |
| `5` | EEF 正在 homing，位置类命令暂不接受 |

Service 返回 `success=true` 表示命令已被 EEF Worker 接受，不表示机械动作已经在 Service 返回前物理完成

### 3.3. OPEN

```bash
ros2 service call \
  /tomato_picker/eef/command \
  tomato_picker_interfaces/srv/CommandEef \
  "{command: 0, value: 0.0}"
```

### 3.4. CLOSE

```bash
ros2 service call \
  /tomato_picker/eef/command \
  tomato_picker_interfaces/srv/CommandEef \
  "{command: 1, value: 0.0}"
```

### 3.5. STOP

```bash
ros2 service call \
  /tomato_picker/eef/command \
  tomato_picker_interfaces/srv/CommandEef \
  "{command: 2, value: 0.0}"
```

### 3.6. SET_POSITION

```bash
ros2 service call \
  /tomato_picker/eef/command \
  tomato_picker_interfaces/srv/CommandEef \
  "{command: 3, value: 0.25}"
```

---

## 4. EEF READY Service

Service：

```text
/tomato_picker/eef/ready
```

类型：

```text
std_srvs/srv/Trigger
```

真机 EEF controller 激活后会异步执行 homing

```bash
ros2 service call \
  /tomato_picker/eef/ready \
  std_srvs/srv/Trigger \
  "{}"
```

状态：

| `success` | `message` | 说明 |
| --- | --- | --- |
| `true` | `READY` | EEF 已完成 homing，可接受位置命令 |
| `false` | `HOMING` | 正在 homing |
| `false` | `UNCONFIGURED` | Worker 未配置 |
| `false` | `INACTIVE` | Worker 未激活 |
| `false` | `ERROR` | EEF 运行错误 |
| `false` | `SHUTDOWN` | Worker 已关闭 |

真机控制前应先确认 `READY`

`eef_mode:=mock` 提供相同 Service 入口，便于上层任务保持统一接口

---

## 5. `PickTarget` Action

Action：

```text
/tomato_picker/task/pick
```

类型：

```text
tomato_picker_interfaces/action/PickTarget
```

用于编排单次采摘任务

### 5.1. 执行流程

默认流程：

```text
APPROACH
↓
MOVE_TARGET
↓
EEF_CLOSE       use_eef=true
↓
RETREAT
↓
PLACE           use_place_pose=true
↓
EEF_OPEN        use_eef=true && use_place_pose=true
↓
HOME            go_home_after_finish=true
```

Task 只调用 `/tomato_picker/motion/move_arm` 与 `/tomato_picker/eef/command`，不直接控制 SerialArm-Core 或 CAN

### 5.2. Goal

| 字段 | 说明 |
| --- | --- |
| `target_pose` | 采摘目标位姿，`frame_id` 必填 |
| `use_approach_pose` | 是否使用显式 `approach_pose` |
| `approach_pose` | 自定义接近位姿 |
| `use_retreat_pose` | 是否使用显式 `retreat_pose` |
| `retreat_pose` | 自定义退出位姿 |
| `use_place_pose` | 是否执行放置阶段 |
| `place_pose` | 放置目标位姿 |
| `use_eef` | 是否执行 EEF CLOSE / OPEN |
| `go_home_after_finish` | 完成后是否回 `home` |
| `approach_distance` | 未提供 approach pose 时沿 planning frame `+Z` 偏移，单位 m |
| `retreat_distance` | 未提供 retreat pose 时沿 planning frame `+Z` 偏移，单位 m |
| `retry_times` | 单个 Pose 运动步骤失败后的额外重试次数 |

`approach_distance <= 0` 时使用 `task.yaml` 的 `default_approach_distance`

`retreat_distance <= 0` 时使用 `task.yaml` 的 `default_retreat_distance`

当前默认值：

```text
approach_distance = 0.08 m
retreat_distance  = 0.10 m
motion velocity   = 0.15
motion acceleration = 0.15
```

### 5.3. Result

| 字段 | 说明 |
| --- | --- |
| `success` | 任务是否成功 |
| `error_code` | 任务错误码 |
| `message` | 结果文本 |
| `completed_steps` | 已完成阶段数量 |
| `canceled` | 是否由取消请求结束 |
| `final_pose` | 最近一次成功 Motion 后的末端位姿 |

错误码：

| 值 | 说明 |
| --- | --- |
| `0` | 成功 |
| `1` | Motion / EEF dependency 不可用 |
| `2` | Motion 步骤失败 |
| `3` | EEF 步骤失败 |
| `4` | 任务被取消 |

### 5.4. Feedback

| 字段 | 说明 |
| --- | --- |
| `current_stage` | 当前阶段编号 |
| `completed_steps` | 已完成步骤数 |
| `total_steps` | 本次任务总步骤数 |
| `stage_text` | 当前阶段文本 |

上层程序应优先使用 `stage_text` 判断当前业务阶段

当前阶段文本包括：

```text
APPROACH
APPROACH_RETRY
MOVE_TARGET
MOVE_TARGET_RETRY
EEF_CLOSE
RETREAT
RETREAT_RETRY
PLACE
PLACE_RETRY
EEF_OPEN
HOME
```

### 5.5. 最小采摘请求

自动生成 approach / retreat，不使用 EEF：

```bash
ros2 action send_goal \
  /tomato_picker/task/pick \
  tomato_picker_interfaces/action/PickTarget \
  "{target_pose: {header: {frame_id: base_link}, pose: {position: {x: 0.40, y: 0.0, z: 0.40}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}, use_eef: false, go_home_after_finish: false, approach_distance: 0.08, retreat_distance: 0.10, retry_times: 0}" \
  --feedback
```

### 5.6. EEF 采摘请求

```bash
ros2 action send_goal \
  /tomato_picker/task/pick \
  tomato_picker_interfaces/action/PickTarget \
  "{target_pose: {header: {frame_id: base_link}, pose: {position: {x: 0.40, y: 0.0, z: 0.40}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}, use_approach_pose: false, use_retreat_pose: false, use_place_pose: false, use_eef: true, go_home_after_finish: true, approach_distance: 0.08, retreat_distance: 0.10, retry_times: 1}" \
  --feedback
```

### 5.7. 完整 Approach / Retreat / Place 请求

```bash
ros2 action send_goal \
  /tomato_picker/task/pick \
  tomato_picker_interfaces/action/PickTarget \
  "{target_pose: {header: {frame_id: base_link}, pose: {position: {x: 0.40, y: 0.0, z: 0.35}, orientation: {w: 1.0}}}, use_approach_pose: true, approach_pose: {header: {frame_id: base_link}, pose: {position: {x: 0.40, y: 0.0, z: 0.43}, orientation: {w: 1.0}}}, use_retreat_pose: true, retreat_pose: {header: {frame_id: base_link}, pose: {position: {x: 0.40, y: 0.0, z: 0.45}, orientation: {w: 1.0}}}, use_place_pose: true, place_pose: {header: {frame_id: base_link}, pose: {position: {x: 0.30, y: -0.20, z: 0.35}, orientation: {w: 1.0}}}, use_eef: true, go_home_after_finish: true, retry_times: 1}" \
  --feedback
```

所有位姿必须在真机执行前单独验证

---

## 6. `SetSceneEnabled` Service

Service：

```text
/tomato_picker/perception/set_scene_enabled
```

类型：

```text
tomato_picker_interfaces/srv/SetSceneEnabled
```

仅在：

```text
start_perception:=true
```

时提供

### 6.1. Request

| 字段 | 说明 |
| --- | --- |
| `enabled` | 是否继续向 Planning Scene 输入过滤点云 |
| `clear_octomap` | 是否同时请求 MoveIt `/clear_octomap` |

`enabled=false` 只停止发布用于 Planning Scene 的 `cloud_filtered`；RGB-D 数据处理和可选 raw/base 发布仍按当前节点配置运行

### 6.2. Response

| 字段 | 说明 |
| --- | --- |
| `success` | 场景切换以及可选 clear 请求是否成功提交 |
| `message` | 结果文本 |

若 `clear_octomap=true` 但 MoveIt `/clear_octomap` 不可用，Service 返回 `success=false`

### 6.3. Enable

```bash
ros2 service call \
  /tomato_picker/perception/set_scene_enabled \
  tomato_picker_interfaces/srv/SetSceneEnabled \
  "{enabled: true, clear_octomap: false}"
```

### 6.4. Disable

```bash
ros2 service call \
  /tomato_picker/perception/set_scene_enabled \
  tomato_picker_interfaces/srv/SetSceneEnabled \
  "{enabled: false, clear_octomap: false}"
```

### 6.5. Disable and Clear Octomap

```bash
ros2 service call \
  /tomato_picker/perception/set_scene_enabled \
  tomato_picker_interfaces/srv/SetSceneEnabled \
  "{enabled: false, clear_octomap: true}"
```

---

## 7. Perception Topics

默认输入：

| Topic | 说明 |
| --- | --- |
| `/camera/wrist/color/image_raw` | RGB 图像 |
| `/camera/wrist/depth_registered/image_raw` | 注册深度图，当前实现要求 `32FC1` |
| `/camera/wrist/depth_registered/camera_info` | 注册深度相机内参 |

默认输出：

| Topic | 说明 |
| --- | --- |
| `/tomato_picker/perception/cloud/raw` | 相机系原始点云 |
| `/tomato_picker/perception/cloud/base` | 转换到 `target_frame` 后的点云 |
| `/tomato_picker/perception/cloud/filtered` | 工作空间裁剪、VoxelGrid、SOR 后的 Planning Scene 点云 |

默认 `target_frame` 为 `base_link`

---

## 8. `TargetObject` Message

类型：

```text
tomato_picker_interfaces/msg/TargetObject
```

字段：

| 字段 | 说明 |
| --- | --- |
| `id` | 目标 ID |
| `class_name` | 类别名称 |
| `confidence` | 识别置信度 |
| `pose` | 目标位姿 |

当前仓库仅定义该公共消息契约，现有默认 `bringup.launch.py` 没有发布固定的 `TargetObject` topic

---

## 9. Bringup 提供的接口

默认：

```bash
ros2 launch tomato_picker_bringup bringup.launch.py
```

启动后预期接口：

```text
/tomato_picker/motion/move_arm
/tomato_picker/eef/command
/tomato_picker/eef/ready
/tomato_picker/task/pick
/tomato_picker/perception/set_scene_enabled
```

其中：

```text
eef_mode:=off
```

时不提供 EEF Service

```text
start_perception:=false
```

时不提供 Planning Scene Service 与 perception topics

机械臂原生 MoveIt / ros2_control 接口由 SerialArm-Core 提供，不在 Tomato-Picker API 中重复定义
