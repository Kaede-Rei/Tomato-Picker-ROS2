from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    robot_profile = LaunchConfiguration("robot_profile")
    serial_port = LaunchConfiguration("serial_port")
    baudrate = LaunchConfiguration("baudrate")
    bus = LaunchConfiguration("bus")
    pose_topic = LaunchConfiguration("pose_topic")
    publish_rate = LaunchConfiguration("publish_rate")

    handeye_bridge = Node(
        package="tomato_picker_bringup",
        executable="handeye_bridge",
        name="handeye_bridge",
        output="screen",
        parameters=[
            {
                "robot_profile": ParameterValue(robot_profile, value_type=str),
                "serial_port": ParameterValue(serial_port, value_type=str),
                "baudrate": ParameterValue(baudrate, value_type=str),
                "bus": ParameterValue(bus, value_type=str),
                "pose_topic": ParameterValue(pose_topic, value_type=str),
                "publish_rate": ParameterValue(publish_rate, value_type=float),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_profile", default_value="dm_arm_gray"),
            DeclareLaunchArgument("serial_port", default_value=""),
            DeclareLaunchArgument("baudrate", default_value=""),
            DeclareLaunchArgument("bus", default_value=""),
            DeclareLaunchArgument("pose_topic", default_value="/arm/pose"),
            DeclareLaunchArgument("publish_rate", default_value="30.0"),
            handeye_bridge,
        ]
    )
