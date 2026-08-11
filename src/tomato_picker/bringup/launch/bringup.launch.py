from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


EEF_MODES = {"off", "mock", "damiao"}


def generate_launch_description():
    robot_profile = LaunchConfiguration("robot_profile")
    serial_port = LaunchConfiguration("serial_port")
    baudrate = LaunchConfiguration("baudrate")
    bus = LaunchConfiguration("bus")
    eef_mode = LaunchConfiguration("eef_mode")
    use_sim_time = LaunchConfiguration("use_sim_time")

    serial_arm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("serial_arm_ros2_control"), "launch", "moveit.launch.py"]
            )
        ),
        launch_arguments={
            "robot_profile": robot_profile,
            "use_sim_time": use_sim_time,
            "serial_port": serial_port,
            "baudrate": baudrate,
            "bus": bus,
        }.items(),
    )

    arm_ready_gate = Node(
        package="tomato_picker_bringup",
        executable="wait_for_arm_ready",
        output="screen",
    )

    eef_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "eef_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-type",
            "tomato_picker_eef/EefController",
            "--param-file",
            PathJoinSubstitution(
                [FindPackageShare("tomato_picker_eef"), "config", "controller.yaml"]
            ),
        ],
    )

    eef_mock = Node(
        package="tomato_picker_eef",
        executable="eef_mock_node",
        output="screen",
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare("tomato_picker_eef"), "config", "mock.yaml"]
            )
        ],
    )

    eef_ready_gate = Node(
        package="tomato_picker_bringup",
        executable="wait_for_eef_ready",
        output="screen",
    )

    motion = Node(
        package="tomato_picker_motion",
        executable="arm_motion_node",
        output="screen",
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare("tomato_picker_motion"), "config", "motion.yaml"]
            ),
            {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
        ],
    )

    motion_ready_gate = Node(
        package="tomato_picker_bringup",
        executable="wait_for_motion_ready",
        output="screen",
    )

    task = Node(
        package="tomato_picker_task",
        executable="pick_task_node",
        output="screen",
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare("tomato_picker_task"), "config", "task.yaml"]
            ),
            {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
        ],
    )

    perception = Node(
        package="tomato_picker_perception",
        executable="cloud_preprocessor",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_perception")),
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare("tomato_picker_perception"), "config", "perception.yaml"]
            ),
            {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
        ],
    )

    wrist_camera = GroupAction(
        condition=IfCondition(LaunchConfiguration("start_wrist_camera")),
        actions=[
            PushRosNamespace("camera"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("orbbec_camera"), "launch", "gemini_330_series.launch.py"]
                    )
                ),
                launch_arguments={
                    "camera_name": "wrist",
                    "depth_registration": "true",
                    "enable_point_cloud": "false",
                    "enable_colored_point_cloud": "false",
                }.items(),
            ),
        ],
    )

    wrist_camera_mount_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        condition=IfCondition(LaunchConfiguration("publish_wrist_camera_mount_tf")),
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", LaunchConfiguration("wrist_camera_mount_frame"),
            "--child-frame-id", LaunchConfiguration("wrist_camera_base_frame"),
        ],
    )

    gui = Node(
        package="tomato_picker_gui",
        executable="wrist_target_gui",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_gui")),
    )

    motion_actions = [
        LogInfo(msg="Tomato Motion starting after hardware readiness"),
        motion,
        motion_ready_gate,
    ]

    app_actions = [
        LogInfo(msg="Tomato Task/Perception/GUI starting after Motion readiness"),
        task,
        perception,
        gui,
    ]

    def after_arm_ready(event, context):
        if event.returncode != 0:
            return [LogInfo(msg=f"ERROR: SerialArm readiness gate failed with exit code {event.returncode}")]

        mode = eef_mode.perform(context)
        if mode not in EEF_MODES:
            return [LogInfo(msg=f"ERROR: unsupported eef_mode '{mode}'")]
        if mode == "off":
            return motion_actions
        if mode == "mock":
            return [LogInfo(msg="SerialArm ready; starting mock EEF"), eef_mock, eef_ready_gate]
        return [LogInfo(msg="SerialArm ready; starting EEF homing"), eef_spawner]

    def after_eef_spawn(event, _context):
        if event.returncode != 0:
            return [LogInfo(msg=f"ERROR: EEF controller spawn failed with exit code {event.returncode}")]
        return [eef_ready_gate]

    def after_eef_ready(event, _context):
        if event.returncode != 0:
            return [LogInfo(msg=f"ERROR: EEF readiness gate failed with exit code {event.returncode}")]
        return motion_actions

    def after_motion_ready(event, _context):
        if event.returncode != 0:
            return [LogInfo(msg=f"ERROR: Motion readiness gate failed with exit code {event.returncode}")]
        return app_actions

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_profile", default_value="dm_arm_gray"),
            DeclareLaunchArgument("serial_port", default_value=""),
            DeclareLaunchArgument("baudrate", default_value=""),
            DeclareLaunchArgument("bus", default_value=""),
            DeclareLaunchArgument("eef_mode", default_value="damiao"),
            DeclareLaunchArgument("start_perception", default_value="true"),
            DeclareLaunchArgument("start_wrist_camera", default_value="false"),
            DeclareLaunchArgument("publish_wrist_camera_mount_tf", default_value="false"),
            DeclareLaunchArgument("wrist_camera_mount_frame", default_value="camera"),
            DeclareLaunchArgument("wrist_camera_base_frame", default_value="camera_link"),
            DeclareLaunchArgument("start_gui", default_value="false"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            SetEnvironmentVariable("TOMATO_PICKER_SERIAL_PORT", serial_port),
            SetEnvironmentVariable("TOMATO_PICKER_BAUDRATE", baudrate),
            SetEnvironmentVariable("TOMATO_PICKER_BUS", bus),
            serial_arm,
            wrist_camera,
            wrist_camera_mount_tf,
            arm_ready_gate,
            RegisterEventHandler(OnProcessExit(target_action=arm_ready_gate, on_exit=after_arm_ready)),
            RegisterEventHandler(OnProcessExit(target_action=eef_spawner, on_exit=after_eef_spawn)),
            RegisterEventHandler(OnProcessExit(target_action=eef_ready_gate, on_exit=after_eef_ready)),
            RegisterEventHandler(OnProcessExit(target_action=motion_ready_gate, on_exit=after_motion_ready)),
        ]
    )
