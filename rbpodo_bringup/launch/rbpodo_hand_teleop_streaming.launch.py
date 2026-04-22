"""
3D Hand Teleoperation with STREAMING SERVO CONTROL.

This uses servo_l for real-time, low-latency control - your hand position
in a 3D cube directly maps to the robot TCP position.

FEATURES:
- MediaPipe 3D hand tracking (run mediapipe_hand_server.py on Mac)
- 30Hz streaming control (vs 1-2Hz with move_l)
- Full 3D workspace: X (depth), Y (left/right), Z (up/down)

Usage:
1. On Mac: python3 mediapipe_hand_server.py
2. In OrbStack: ros2 launch rbpodo_bringup rbpodo_hand_teleop_streaming.launch.py robot_ip:=192.168.0.100 enable_hand_teleop:=true

Hand teleop nodes are off by default; pass enable_hand_teleop:=true to run them.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, 
    TextSubstitution, PythonExpression
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
        # ===================== ARGUMENTS =====================
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
        
        # Hand teleop arguments
        DeclareLaunchArgument("workspace_size", default_value="0.05",
                            description="Workspace cube half-size in meters (e.g., 0.05 = +/- 5cm)"),
        DeclareLaunchArgument("control_rate", default_value="30.0",
                            description="Servo control rate in Hz (default 30)"),
        DeclareLaunchArgument("enable_debug", default_value="true"),
        
        # Servo control tuning (for smooth motion)
        DeclareLaunchArgument("servo_t1", default_value="0.08",
                            description="Servo arrival time (sec, higher = smoother)"),
        DeclareLaunchArgument("servo_t2", default_value="0.15",
                            description="Servo hold time (sec)"),
        DeclareLaunchArgument("servo_gain", default_value="0.6",
                            description="Servo speed tracking gain (lower = smoother)"),
        DeclareLaunchArgument("servo_alpha", default_value="0.2",
                            description="Servo low-pass filter (lower = smoother)"),
        DeclareLaunchArgument("smoothing_factor", default_value="0.3",
                            description="Position smoothing 0-1 (lower = smoother)"),
        
        # Mac camera bridge
        DeclareLaunchArgument("use_mac_camera", default_value="true"),
        DeclareLaunchArgument("mac_ip", default_value="host.internal"),
        DeclareLaunchArgument("mac_hand_port", default_value="9998",
                            description="Port for MediaPipe hand data"),
        DeclareLaunchArgument("use_python_node", default_value="false",
                            description="Use Python node instead of C++ (easier debugging)"),
        DeclareLaunchArgument(
            "enable_hand_teleop",
            default_value="false",
            description="Set true to start hand bridge / streaming teleop nodes",
        ),

        # ===================== ROBOT NODES =====================
        
        # ros2_control node
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

        # Robot state publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="both",
            parameters=[{"robot_description": robot_description}],
        ),

        # Joint state publisher
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            parameters=[{"source_list": ["rbpodo/joint_states"], "rate": 50}],
        ),

        # Joint state broadcaster
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster"],
            output="screen",
        ),

        # ===================== HAND TELEOP NODES (disabled unless enable_hand_teleop:=true) =====================

        # Hand position bridge (receives 3D hand data from Mac)
        Node(
            package="rbpodo_hand_teleop",
            executable="hand_position_bridge.py",
            name="hand_position_bridge",
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration("use_mac_camera"), "' == 'true' and '",
                LaunchConfiguration("enable_hand_teleop"), "' == 'true'",
            ])),
            parameters=[{
                "mac_ip": LaunchConfiguration("mac_ip"),
                "mac_port": LaunchConfiguration("mac_hand_port"),
                "position_topic": "/hand/position",
                "detected_topic": "/hand/detected",
            }],
            output="screen",
        ),

        # STREAMING hand teleop node - C++ version (default)
        Node(
            package="rbpodo_hand_teleop",
            executable="hand_teleop_streaming_node",
            name="hand_teleop_streaming_node",
            condition=IfCondition(
                PythonExpression([
                    "'", LaunchConfiguration("enable_hand_teleop"), "' == 'true' and '",
                    LaunchConfiguration("use_python_node"), "' != 'true'",
                ])
            ),
            parameters=[{
                "workspace_size": LaunchConfiguration("workspace_size"),
                "control_rate": LaunchConfiguration("control_rate"),
                "enable_debug": LaunchConfiguration("enable_debug"),
                "servo_t1": LaunchConfiguration("servo_t1"),
                "servo_t2": LaunchConfiguration("servo_t2"),
                "servo_gain": LaunchConfiguration("servo_gain"),
                "servo_alpha": LaunchConfiguration("servo_alpha"),
                "smoothing_factor": LaunchConfiguration("smoothing_factor"),
            }],
            output="screen",
        ),

        # STREAMING hand teleop node - Python version (easier debugging)
        Node(
            package="rbpodo_hand_teleop",
            executable="hand_teleop_streaming.py",
            name="hand_teleop_streaming_py",
            condition=IfCondition(
                PythonExpression([
                    "'", LaunchConfiguration("enable_hand_teleop"), "' == 'true' and '",
                    LaunchConfiguration("use_python_node"), "' == 'true'",
                ])
            ),
            parameters=[{
                "workspace_size": LaunchConfiguration("workspace_size"),
                "control_rate": LaunchConfiguration("control_rate"),
                "enable_debug": LaunchConfiguration("enable_debug"),
                "servo_t1": LaunchConfiguration("servo_t1"),
                "servo_t2": LaunchConfiguration("servo_t2"),
                "servo_gain": LaunchConfiguration("servo_gain"),
                "servo_alpha": LaunchConfiguration("servo_alpha"),
                "smoothing_factor": LaunchConfiguration("smoothing_factor"),
            }],
            output="screen",
        ),
    ])
