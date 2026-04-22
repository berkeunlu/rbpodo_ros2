from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "image_topic",
            default_value="/camera/image_raw",
            description="Camera image topic"
        ),
        DeclareLaunchArgument(
            "workspace_size",
            default_value="0.05",
            description="Workspace cube half-size in meters (0.05 = +/- 5cm)"
        ),
        DeclareLaunchArgument(
            "move_speed",
            default_value="0.05",
            description="TCP move speed in m/s"
        ),
        DeclareLaunchArgument(
            "move_acceleration",
            default_value="0.1",
            description="TCP move acceleration in m/s^2"
        ),
        DeclareLaunchArgument(
            "control_rate",
            default_value="5.0",
            description="Control loop rate in Hz (slower = smoother)"
        ),
        DeclareLaunchArgument(
            "enable_debug",
            default_value="false",
            description="Enable debug logging"
        ),
        DeclareLaunchArgument(
            "use_hsv",
            default_value="false",
            description="Use HSV color space for hand tracking"
        ),
        DeclareLaunchArgument(
            "threshold_low",
            default_value="200",
            description="Brightness threshold (0-255)"
        ),
        DeclareLaunchArgument(
            "enable_hand_teleop",
            default_value="false",
            description="Set true to start hand_teleop_node",
        ),
        Node(
            package="rbpodo_hand_teleop",
            executable="hand_teleop_node",
            name="hand_teleop_node",
            condition=IfCondition(LaunchConfiguration("enable_hand_teleop")),
            parameters=[{
                "image_topic": LaunchConfiguration("image_topic"),
                "workspace_size": LaunchConfiguration("workspace_size"),
                "move_speed": LaunchConfiguration("move_speed"),
                "move_acceleration": LaunchConfiguration("move_acceleration"),
                "control_rate": LaunchConfiguration("control_rate"),
                "enable_debug": LaunchConfiguration("enable_debug"),
                "use_hsv": LaunchConfiguration("use_hsv"),
                "threshold_low": LaunchConfiguration("threshold_low"),
            }],
            output="screen",
        ),
    ])
