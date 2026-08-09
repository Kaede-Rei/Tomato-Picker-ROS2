# Tomato-Picker-ROS2

Tomato-Picker-ROS2 是基于 ROS 2 Humble、MoveIt 2 与 SerialArm-Core 的番茄采摘机器人应用工作区，SerialArm-Core 提供通用机械臂模型、控制、动力学、安全、Hardware Backend 与 Transport，Tomato-Picker 在其上提供感知、运动、EEF 和采摘任务能力

## System Capabilities

| Capability | Interface | Status |
| --- | --- | --- |
| Home target | `HOME` | Available |
| Joint target | `JOINT` | Available |
| Pose target | `POSE` | Available |
| Cartesian line | `LINE` | Available |
| EEF open / close / stop | `OPEN` / `CLOSE` / `STOP` | Available |
| EEF normalized position | `SET_POSITION` | Available |
| Mock EEF | `eef_mode:=mock` | Available |
| DM4310 EEF | `eef_mode:=damiao` | Available |
| Pick orchestration | `PickTarget.action` | Available |
| RGB-D cloud preprocessing | `/tomato_picker/perception/cloud/filtered` | Available |

## Workspace

```text
Tomato-Picker-ROS2/
├── repos/
│   └── serial_arm.repos
├── src/
│   ├── SerialArm-Core/
│   └── tomato_picker/
│       ├── interfaces/
│       ├── perception/
│       ├── motion/
│       ├── eef/
│       ├── task/
│       └── bringup/
└── tools/
```

`src/SerialArm-Core` 通过 `repos/serial_arm.repos` 获取，作为独立依赖维护，不在 Tomato-Picker 中复制实现

## Architecture

```text
Perception
    │
    ↓
PickTarget Task
    │
    ├───────────────┐
    ↓               ↓
MoveArm          CommandEef
    ↓               ↓
MoveIt 2         EEF Controller
    ↓               ↓
ros2_control     EEF Worker
    │               │
    └───────┬───────┘
            ↓
      SerialArm-Core
            ↓
      DM-Arm + DM4310
```

ARM 与 EEF 复用 SerialArm-Core Transport，真实 EEF 与机械臂在同一 `controller_manager` 进程中共享 `main_can`，EEF Worker 在非实时线程完成硬件操作，不阻塞 ros2_control realtime loop

## Quick Start

### 1. Prepare Workspace

```bash
git clone <Tomato-Picker-ROS2-repository-url>
cd Tomato-Picker-ROS2
source /opt/ros/humble/setup.bash
vcs import src < repos/serial_arm.repos
```

若系统尚未安装 vcstool，可执行 `sudo apt install python3-vcstool`，随后安装依赖：

```bash
rosdep install \
  --from-paths src \
  --ignore-src \
  -r -y \
  --rosdistro humble
```

### 2. Build

```bash
colcon build
```

该命令同时构建 SerialArm-Core 与 Tomato-Picker，开发阶段可使用 `colcon build --symlink-install`

### 3. Source Workspace

```bash
# 当前已加载 ROS 2 的终端
source install/setup.bash

# 新终端
source /opt/ros/humble/setup.bash
source <workspace>/install/setup.bash
```

### 4. Check SerialArm

```bash
ros2 pkg prefix serial_arm_core
ros2 pkg prefix serial_arm_ros2_control
ros2 pkg prefix dm_arm_description
python3 -c "import serial_arm; print(serial_arm.__file__)"
```

先进行不连接真实 Hardware Backend 的模型检查：

```bash
ros2 launch serial_arm_ros2_control display.launch.py \
  robot_profile:=dm_arm_gray
```

确认 URDF、TF 与 joint direction 后，再启动 ARM hardware：

```bash
ros2 launch serial_arm_ros2_control hardware.launch.py \
  robot_profile:=dm_arm_gray

ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 topic echo /joint_states
```

机械臂状态未知时不要直接执行完整 PickTarget

### 5. Launch Tomato-Picker

```bash
ros2 launch tomato_picker_bringup system.launch.py \
  robot_profile:=dm_arm_gray \
  eef_mode:=off \
  start_perception:=false
```

## Runtime Modes

| Argument | Behavior | Use case |
| --- | --- | --- |
| `eef_mode:=off` | 不启动 EEF | ARM、Motion、Task 基础联调 |
| `eef_mode:=mock` | 启动 Mock EEF，不连接 DM4310 | PickTarget 软件流程测试 |
| `eef_mode:=damiao` | 加载真实 DM4310 EEF Controller，与机械臂共享 `main_can`，启动后执行 homing | EEF 与整机真机测试 |
| `start_perception:=false` | 不启动点云预处理 | 早期 ARM、EEF、Task 调试 |
| `start_perception:=true` | 启动当前 RGB-D 点云能力 | 基础能力稳定后的感知联调 |

真实 EEF 第一次启动前必须确认 homing 方向、机械止挡和操作区域安全

## Debugging

不要一开始直接调完整 PickTarget，应从 SerialArm 和硬件层开始逐层确认 capability

```bash
ros2 pkg prefix tomato_picker_motion
ros2 pkg prefix tomato_picker_eef
ros2 pkg prefix tomato_picker_task
ros2 action info /tomato_picker/motion/move_arm
ros2 action info /tomato_picker/task/pick
ros2 service type /tomato_picker/eef/command
```

### SerialArm / Hardware

复用 Quick Start 中的 controller、hardware interface 与 `/joint_states` 检查，真实 EEF 模式下应能看到 `eef_controller`，具体 state 以现场输出为准

### Motion / MoveIt

| Command | Behavior |
| --- | --- |
| `HOME` | 回到配置的 named target |
| `JOINT` | 规划 joint-space target |
| `POSE` | 使用完整 `PoseStamped` 进行末端位姿规划 |
| `LINE` | 生成 Cartesian path，`frame_id` 必须等于 MoveGroup 当前 pose reference frame |

HOME 规划示例，确认规划安全后再将 `execute` 改为 `true`：

```bash
ros2 action send_goal /tomato_picker/motion/move_arm \
  tomato_picker_interfaces/action/MoveArm \
  "{command_type: 0, velocity_scale: 0.1, acceleration_scale: 0.1, execute: false}"
```
JOINT 规划示例：

```bash
ros2 action send_goal /tomato_picker/motion/move_arm \
  tomato_picker_interfaces/action/MoveArm \
  "{command_type: 1, joint_names: [joint1, joint2, joint3, joint4, joint5, joint6], joints: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], velocity_scale: 0.1, acceleration_scale: 0.1, execute: false}"
```

### EEF

`CommandEef.srv` 命令值为 `OPEN=0`、`CLOSE=1`、`STOP=2`、`SET_POSITION=3`

```bash
ros2 service call /tomato_picker/eef/command tomato_picker_interfaces/srv/CommandEef "{command: 0, value: 0.0}"
ros2 service call /tomato_picker/eef/command tomato_picker_interfaces/srv/CommandEef "{command: 3, value: 0.25}"
```

`eef_controller` active 不代表 homing 已完成，真实 EEF 应等待日志 `EEF homing complete; worker READY at configured rate`

### Task

以下示例保持 `use_eef=false`，目标位姿必须替换为已验证的安全值：

```bash
ros2 action send_goal /tomato_picker/task/pick \
  tomato_picker_interfaces/action/PickTarget \
  "{target_pose: {header: {frame_id: base_link}, pose: {position: {x: 0.40, y: 0.0, z: 0.40}, orientation: {w: 1.0}}}, use_approach_pose: false, use_retreat_pose: false, use_place_pose: false, use_eef: false, go_home_after_finish: false, approach_distance: 0.08, retreat_distance: 0.10, retry_times: 0}"
```
确认 Motion 与 Mock 或真实 EEF 独立正常后，再使用 `use_eef=true`

### Perception

```bash
ros2 topic hz /tomato_picker/perception/cloud/filtered
ros2 service call /tomato_picker/perception/set_scene_enabled \
  tomato_picker_interfaces/srv/SetSceneEnabled \
  "{enabled: true, clear_octomap: false}"
```
输入 topic 为 `/camera/wrist/color/image_raw`、`/camera/wrist/depth_registered/image_raw` 与 `/camera/wrist/depth_registered/camera_info`

## Recommended Bringup Order

1. `colcon build`
2. SerialArm package 与 Python binding 检查
3. `display.launch.py` 检查 URDF、TF、joint direction
4. ARM hardware 检查 `/joint_states` 与 controller
5. Tomato-Picker 使用 `eef_mode=off`、`start_perception=false`
6. MoveArm HOME
7. MoveArm JOINT 小幅运动
8. MoveArm POSE
9. EEF mock
10. PickTarget `use_eef=false`
11. EEF damiao 与 homing
12. EEF `SET_POSITION` 小幅命令
13. EEF `OPEN`、`CLOSE`、`STOP`
14. ARM 与 EEF 并发测试
15. PickTarget `use_eef=true`
16. 启用 perception
17. 完整番茄采摘流程

## Packages

| Package | Responsibility |
| --- | --- |
| `tomato_picker_interfaces` | ROS contracts |
| `tomato_picker_perception` | Perception capability |
| `tomato_picker_motion` | MoveIt motion capability |
| `tomato_picker_eef` | DM4310 与 Mock EEF capability |
| `tomato_picker_task` | PickTarget orchestration |
| `tomato_picker_bringup` | System launch |
