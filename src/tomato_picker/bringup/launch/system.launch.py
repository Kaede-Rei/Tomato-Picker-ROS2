from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

sys.path.append(
    str(Path(get_package_share_directory("serial_arm_ros2_control")) / "scripts")
)
from profile_utils import load_profile, require_moveit_package


EEF_MODES = {"off", "mock", "damiao"}
ARM_HARDWARE_COMPONENT = "dm_arm"


def _manager_identity(manager_fqn):
    normalized = manager_fqn if manager_fqn.startswith("/") else f"/{manager_fqn}"
    parts = normalized.strip("/").split("/")
    node_name = parts[-1]
    namespace = "/" + "/".join(parts[:-1]) if len(parts) > 1 else ""
    return normalized, node_name, namespace


def _continue_on_success(label, actions):
    def callback(event, context):
        del context
        if event.returncode == 0:
            return actions
        reason = f"{label} failed with exit code {event.returncode}"
        return [LogInfo(msg=f"ERROR: {reason}"), EmitEvent(event=Shutdown(reason=reason))]

    return callback


def _shutdown_on_failure(label):
    def callback(event, context):
        del context
        if event.returncode == 0:
            return []
        reason = f"{label} failed with exit code {event.returncode}"
        return [LogInfo(msg=f"ERROR: {reason}"), EmitEvent(event=Shutdown(reason=reason))]

    return callback


def resolve_system(context):
    robot_profile = LaunchConfiguration("robot_profile").perform(context)
    eef_mode = LaunchConfiguration("eef_mode").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() in (
        "true",
        "1",
        "yes",
    )
    manager_fqn, manager_node_name, manager_namespace = _manager_identity(
        LaunchConfiguration("controller_manager_name").perform(context)
    )
    if manager_node_name != "controller_manager":
        raise RuntimeError(
            "controller_manager_name may change the namespace, but its basename must remain "
            "'controller_manager' so dynamically loaded controllers do not inherit a global "
            "node-name remap"
        )

    if eef_mode not in EEF_MODES:
        raise RuntimeError(
            f"Unsupported eef_mode '{eef_mode}'; expected one of {sorted(EEF_MODES)}"
        )

    profile = load_profile(robot_profile)
    moveit_package = require_moveit_package(profile, robot_profile)
    robot_description = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                profile["ros2_control_xacro_path"],
                " config_file:=",
                profile["core_config_path"],
                " hardware_plugin:=",
                profile["hardware_plugin"],
                " hardware_config:=",
                profile["hardware_config_path"],
            ]
        ),
        value_type=str,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": use_sim_time,
            }
        ],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=manager_namespace,
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            profile["controllers_path"],
            {
                "hardware_components_initial_state.inactive": [
                    ARM_HARDWARE_COMPONENT
                ],
                "use_sim_time": use_sim_time,
            },
        ],
        remappings=[("~/robot_description", "/robot_description")],
    )

    arm_hardware_spawner = Node(
        package="controller_manager",
        executable="hardware_spawner",
        name="dm_arm_hardware_spawner",
        output="screen",
        arguments=[
            ARM_HARDWARE_COMPONENT,
            "--controller-manager",
            manager_fqn,
            "--controller-manager-timeout",
            "30.0",
            "--activate",
        ],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="arm_controller_spawner",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "joint_trajectory_controller",
            "--controller-manager",
            manager_fqn,
            "--controller-manager-timeout",
            "30.0",
            "--activate-as-group",
        ],
    )

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare(moveit_package), "launch", "move_group.launch.py"]
            )
        )
    )
    moveit_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare(moveit_package), "launch", "moveit_rviz.launch.py"]
            )
        )
    )

    motion = Node(
        package="tomato_picker_motion",
        executable="arm_motion_node",
        name="arm_motion_node",
        output="screen",
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare("tomato_picker_motion"), "config", "motion.yaml"]
            ),
            {"use_sim_time": use_sim_time},
        ],
    )

    task = Node(
        package="tomato_picker_task",
        executable="pick_task_node",
        name="pick_task_node",
        output="screen",
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare("tomato_picker_task"), "config", "task.yaml"]
            ),
            {"use_sim_time": use_sim_time},
        ],
    )

    perception = Node(
        package="tomato_picker_perception",
        executable="cloud_preprocessor",
        name="cloud_preprocessor",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_perception")),
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare("tomato_picker_perception"), "config", "perception.yaml"]
            ),
            {"use_sim_time": use_sim_time},
        ],
    )

    runtime_actions = [
        LogInfo(msg="ARM controllers active; starting MoveIt, Motion and Task"),
        move_group,
        moveit_rviz,
        motion,
        task,
        perception,
    ]

    actions = [
        robot_state_publisher,
        controller_manager,
        RegisterEventHandler(
            OnProcessExit(
                target_action=arm_hardware_spawner,
                on_exit=_continue_on_success(
                    "ARM hardware activation",
                    [
                        LogInfo(msg="ARM hardware active; starting ARM controllers"),
                        arm_controller_spawner,
                    ],
                ),
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=arm_controller_spawner,
                on_exit=_continue_on_success(
                    "ARM controller activation", runtime_actions
                ),
            )
        ),
    ]

    if eef_mode == "damiao":
        eef_damiao = Node(
            package="controller_manager",
            executable="spawner",
            name="eef_controller_spawner",
            output="screen",
            arguments=[
                "eef_controller",
                "--controller-manager",
                manager_fqn,
                "--controller-manager-timeout",
                "30.0",
                "--controller-type",
                "tomato_picker_eef/EefController",
                "--param-file",
                PathJoinSubstitution(
                    [FindPackageShare("tomato_picker_eef"), "config", "controller.yaml"]
                ),
            ],
        )
        eef_ready_gate = Node(
            package="tomato_picker_bringup",
            executable="wait_for_eef_ready",
            name="eef_ready_gate",
            output="screen",
            parameters=[
                {
                    "ready_service": "/tomato_picker/eef/ready",
                    "timeout_sec": 15.0,
                    "poll_period_sec": 0.1,
                    "use_sim_time": use_sim_time,
                }
            ],
        )
        actions.extend(
            [
                LogInfo(msg="ARM hardware held INACTIVE; starting EEF homing first"),
                eef_damiao,
                eef_ready_gate,
                RegisterEventHandler(
                    OnProcessExit(
                        target_action=eef_damiao,
                        on_exit=_shutdown_on_failure("EEF controller spawner"),
                    )
                ),
                RegisterEventHandler(
                    OnProcessExit(
                        target_action=eef_ready_gate,
                        on_exit=_continue_on_success(
                            "EEF initialization",
                            [
                                LogInfo(msg="EEF READY; activating ARM hardware"),
                                arm_hardware_spawner,
                            ],
                        ),
                    )
                ),
            ]
        )
        return actions

    if eef_mode == "mock":
        actions.append(
            Node(
                package="tomato_picker_eef",
                executable="eef_mock_node",
                name="eef_mock_node",
                output="screen",
                parameters=[
                    PathJoinSubstitution(
                        [FindPackageShare("tomato_picker_eef"), "config", "mock.yaml"]
                    ),
                    {"use_sim_time": use_sim_time},
                ],
            )
        )

    actions.extend(
        [
            LogInfo(msg="No real EEF homing gate; activating ARM hardware"),
            arm_hardware_spawner,
        ]
    )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_profile", default_value="dm_arm_gray"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("start_perception", default_value="true"),
            DeclareLaunchArgument("eef_mode", default_value="off"),
            DeclareLaunchArgument(
                "controller_manager_name", default_value="/controller_manager"
            ),
            OpaqueFunction(function=resolve_system),
        ]
    )
