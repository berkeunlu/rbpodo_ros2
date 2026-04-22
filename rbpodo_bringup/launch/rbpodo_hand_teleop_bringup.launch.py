"""
Unified bringup for hand teleoperation using native rbpodo control.

NO MoveIt required - uses rbpodo's SystemState for TCP pose and move_l action.
The robot TCP pose is obtained directly from /rbpodo_hardware/system_state topic.

Hand teleop nodes are disabled by default; pass enable_hand_teleop:=true to start them.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
    TextSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Robot arguments
    robot_ip = LaunchConfiguration("robot_ip")
    model_id = LaunchConfiguration("model_id")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    cb_simulation = LaunchConfiguration("cb_simulation")
    model_path = LaunchConfiguration("model_path")

    # Robot description
    robot_description = Command([
        FindExecutable(name="xacro"), " ",
        model_path,
        " cb_simulation:=", cb_simulation,
        " robot_ip:=", robot_ip,
        " use_fake_hardware:=", use_fake_hardware,
        " fake_sensor_commands:=false",
    ])

    robot_controllers = PathJoinSubstitution([
        FindPackageShare("rbpodo_bringup"),
        "config",
        "controllers.yaml",
    ])

    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument("robot_ip", default_value="192.168.0.100"),
        DeclareLaunchArgument("model_id", default_value="rb10_1300e"),
        DeclareLaunchArgument("model_path", 
                            default_value=[
                                TextSubstitution(
                                    text=os.path.join(
                                        get_package_share_directory("rbpodo_description"),
                                        "robots", ""
                                    )
                                ),
                                model_id,
                                TextSubstitution(text=".urdf.xacro"),
                            ],
                            description="Model path (xacro)"),
        DeclareLaunchArgument("use_fake_hardware", default_value="false"),
        DeclareLaunchArgument("cb_simulation", default_value="false"),
        DeclareLaunchArgument("workspace_size", default_value="0.05",
                            description="Workspace cube half-size in meters (max 0.10)"),
        DeclareLaunchArgument("move_speed", default_value="0.05",
                            description="TCP move speed in m/s (50mm/s default)"),
        DeclareLaunchArgument("enable_debug", default_value="false"),
        DeclareLaunchArgument("use_mac_camera", default_value="false"),
        DeclareLaunchArgument("mac_ip", default_value="host.internal"),
        DeclareLaunchArgument(
            "enable_hand_teleop",
            default_value="false",
            description="Set true to start hand teleop and optional Mac camera bridge",
        ),

        # ros2_control node - this starts RobotNode with move_l action & system_state topic
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[robot_controllers],
            remappings=[
                ("joint_states", "rbpodo/joint_states"),
                ("~/robot_description", "/robot_description"),
            ],
            output="both",
            on_exit=Shutdown(),
        ),

        # Robot state publisher - publishes TF frames including tcp
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="both",
            parameters=[{"robot_description": robot_description}],
        ),

        # Joint state publisher - combines joint states
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            parameters=[{"source_list": ["rbpodo/joint_states"], "rate": 30}],
        ),

        # Spawn joint state broadcaster ONLY (no trajectory controller - we use move_l action)
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster"],
            output="screen",
        ),

        # Hand teleop node (disabled by default; pass enable_hand_teleop:=true to use)
        Node(
            package="rbpodo_hand_teleop",
            executable="hand_teleop_node",
            name="hand_teleop_node",
            condition=IfCondition(LaunchConfiguration("enable_hand_teleop")),
            parameters=[{
                "image_topic": "/camera/image_raw",
                "workspace_size": LaunchConfiguration("workspace_size"),
                "move_speed": LaunchConfiguration("move_speed"),
                "enable_debug": LaunchConfiguration("enable_debug"),
            }],
            output="screen",
        ),

        # Mac camera bridge (optional; requires enable_hand_teleop:=true)
        Node(
            package="rbpodo_hand_teleop",
            executable="mac_camera_bridge.py",
            name="mac_camera_bridge",
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration("use_mac_camera"), "' == 'true' and '",
                LaunchConfiguration("enable_hand_teleop"), "' == 'true'",
            ])),
            parameters=[{
                "mac_ip": LaunchConfiguration("mac_ip"),
                "mac_port": 9999,
                "image_topic": "/camera/image_raw",
            }],
            output="screen",
        ),
    ])
