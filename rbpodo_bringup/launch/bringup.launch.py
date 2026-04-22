"""Full rbpodo bringup: arm + ZED Mini + (optional) RTAB-Map SLAM.

Layers:
  1. Hardware        rbpodo.launch.py (ros2_control + robot_state_publisher)
  2. Perception      zed_wrapper (publish_tf:=false — arm RSP owns camera TF)
  3. SLAM            rtabmap_zedm.launch.py (optional)
  4. Static TFs      world -> link0 [-> base_link]

Examples:
  Real robot + ZED + SLAM (rtabmap VO):
    ros2 launch rbpodo_bringup bringup.launch.py \\
        model_id:=rb10_1300e robot_ip:=192.168.0.100

  Fake hardware + ZED + SLAM:
    ros2 launch rbpodo_bringup bringup.launch.py \\
        model_id:=rb10_1300e use_fake_hardware:=true

  No SLAM, just robot + camera:
    ros2 launch rbpodo_bringup bringup.launch.py enable_slam:=false

  ZED VIO odom instead of rtabmap VO:
    ros2 launch rbpodo_bringup bringup.launch.py odom_source:=zed

  Localization on prior map:
    ros2 launch rbpodo_bringup bringup.launch.py \\
        localization:=true delete_db:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ── Args ─────────────────────────────────────────────────────────────
    robot_ip           = LaunchConfiguration('robot_ip')
    model_id           = LaunchConfiguration('model_id')
    use_fake_hardware  = LaunchConfiguration('use_fake_hardware')
    cb_simulation      = LaunchConfiguration('cb_simulation')
    use_rviz           = LaunchConfiguration('use_rviz')

    enable_camera      = LaunchConfiguration('enable_camera')
    zed_camera_model   = LaunchConfiguration('zed_camera_model')
    zed_camera_name    = LaunchConfiguration('zed_camera_name')

    enable_slam        = LaunchConfiguration('enable_slam')
    odom_source        = LaunchConfiguration('odom_source')
    localization       = LaunchConfiguration('localization')
    delete_db          = LaunchConfiguration('delete_db')
    database_path      = LaunchConfiguration('database_path')

    # ZED publish_tf must be false when rtabmap VO is active (avoid TF loop
    # via odom->zedm_camera_link + link0->...->zedm_camera_link via FK).
    # When odom_source=zed, ZED must publish odom TF so the map tree exists.
    zed_publish_tf = PythonExpression(
        ["'true' if '", odom_source, "' == 'zed' else 'false'"]
    )

    # ── Includes ─────────────────────────────────────────────────────────
    arm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('rbpodo_bringup'), 'launch', 'rbpodo.launch.py'
        ])),
        launch_arguments={
            'robot_ip': robot_ip,
            'model_id': model_id,
            'use_fake_hardware': use_fake_hardware,
            'cb_simulation': cb_simulation,
            'use_rviz': 'false',
        }.items(),
    )

    rviz_config = os.path.join(
        get_package_share_directory('rbpodo_description'), 'rviz', 'urdf.rviz'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='log',
        condition=IfCondition(use_rviz),
    )

    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('zed_wrapper'), 'launch', 'zed_camera.launch.py'
        ])),
        launch_arguments={
            'camera_model':     zed_camera_model,
            'camera_name':      zed_camera_name,
            'publish_urdf':     'false',
            'publish_tf':       zed_publish_tf,
            'publish_map_tf':   'false',
            'publish_imu_tf':   'false',
        }.items(),
        condition=IfCondition(enable_camera),
    )

    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('rbpodo_slam'), 'launch', 'rtabmap_zedm.launch.py'
        ])),
        launch_arguments={
            'zed_namespace':  zed_camera_name,
            'odom_source':    odom_source,
            'localization':   localization,
            'delete_db':      delete_db,
            'database_path':  database_path,
            'rviz':           'false',
            'rtabmap_viz':    'false',
        }.items(),
        condition=IfCondition(enable_slam),
    )

    # ── Static TFs ───────────────────────────────────────────────────────
    # world -> link0: root anchor (only if rtabmap not running, else rtabmap
    # publishes map->odom->link0 and this would create a loop).
    static_world_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_link0',
        arguments=['--x','0','--y','0','--z','0',
                   '--qx','0','--qy','0','--qz','0','--qw','1',
                   '--frame-id','world','--child-frame-id','link0'],
        output='log',
        condition=UnlessCondition(enable_slam),
    )

    # link0 -> base_link: MoveIt/other tools often expect base_link.
    # Not adding to URDF keeps SRDF/URDF unchanged.
    static_baselink_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='link0_to_base_link',
        arguments=['--x','0','--y','0','--z','0',
                   '--qx','0','--qy','0','--qz','0','--qw','1',
                   '--frame-id','link0','--child-frame-id','base_link'],
        output='log',
    )

    return LaunchDescription([
        # Robot
        DeclareLaunchArgument('robot_ip',          default_value='10.0.2.7'),
        DeclareLaunchArgument('model_id',          default_value='rb10_1300e'),
        DeclareLaunchArgument('use_fake_hardware', default_value='false'),
        DeclareLaunchArgument('cb_simulation',     default_value='false'),
        DeclareLaunchArgument('use_rviz',          default_value='true'),

        # Camera
        DeclareLaunchArgument('enable_camera',    default_value='true'),
        DeclareLaunchArgument('zed_camera_model', default_value='zedm'),
        DeclareLaunchArgument('zed_camera_name',  default_value='zedm'),

        # SLAM
        DeclareLaunchArgument('enable_slam',   default_value='true'),
        DeclareLaunchArgument('odom_source',   default_value='rtabmap',
                              description='rtabmap (VO, frame=link0) or zed (VIO).'),
        DeclareLaunchArgument('localization',  default_value='false',
                              description='Localize on existing map; no new mapping.'),
        DeclareLaunchArgument('delete_db',     default_value='true',
                              description='Wipe map db on mapping start.'),
        DeclareLaunchArgument('database_path', default_value='~/.ros/rtabmap_zedm.db'),

        arm_launch,
        zed_launch,
        rtabmap_launch,
        static_world_tf,
        static_baselink_tf,
        rviz_node,
    ])
