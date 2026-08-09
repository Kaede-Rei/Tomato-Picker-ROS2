from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


EEF_MODES = {"off", "mock", "damiao"}


def validate_eef_mode(context):
    eef_mode = LaunchConfiguration("eef_mode").perform(context)
    if eef_mode not in EEF_MODES:
        raise RuntimeError(
            f"Unsupported eef_mode '{eef_mode}'; expected one of {sorted(EEF_MODES)}"
        )
    return []


def generate_launch_description():
    robot_profile = LaunchConfiguration("robot_profile")
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_perception = LaunchConfiguration("start_perception")
    eef_mode = LaunchConfiguration("eef_mode")
    controller_manager_name = LaunchConfiguration("controller_manager_name")

    serial_arm_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("serial_arm_ros2_control"), "launch", "moveit.launch.py"]
            )
        ),
        launch_arguments={
            "robot_profile": robot_profile,
            "use_sim_time": use_sim_time,
            "controller_manager_name": controller_manager_name,
        }.items(),
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

    eef_mock = Node(
        package="tomato_picker_eef",
        executable="eef_mock_node",
        name="eef_mock_node",
        output="screen",
        condition=IfCondition(PythonExpression(["'", eef_mode, "' == 'mock'"])),
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare("tomato_picker_eef"), "config", "mock.yaml"]
            ),
            {"use_sim_time": use_sim_time},
        ],
    )

    eef_damiao = Node(
        package="controller_manager",
        executable="spawner",
        name="eef_controller_spawner",
        output="screen",
        condition=IfCondition(PythonExpression(["'", eef_mode, "' == 'damiao'"])),
        arguments=[
            "eef_controller",
            "--controller-manager",
            controller_manager_name,
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
        condition=IfCondition(start_perception),
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare("tomato_picker_perception"), "config", "perception.yaml"]
            ),
            {"use_sim_time": use_sim_time},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_profile", default_value="dm_arm_gray"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("start_perception", default_value="true"),
            DeclareLaunchArgument("eef_mode", default_value="off"),
            DeclareLaunchArgument(
                "controller_manager_name", default_value="/controller_manager"
            ),
            OpaqueFunction(function=validate_eef_mode),
            serial_arm_moveit,
            motion,
            eef_mock,
            eef_damiao,
            task,
            perception,
        ]
    )
