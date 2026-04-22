"""RTAB-Map RGBD SLAM launch for Intel RealSense D435 on rbpodo.

Assumes the RealSense driver is already running and publishing:
  - RGB:           /camera/camera/color/image_raw
  - Aligned depth: /camera/camera/aligned_depth_to_color/image_raw
  - Camera info:   /camera/camera/color/camera_info

Aligned depth is required. Enable it in the RealSense config:
  align_depth.enable: True

D435 has no IMU, so visual odometry only.

Example use:
  ros2 launch rbpodo_slam rtabmap.launch.py
  ros2 launch rbpodo_slam rtabmap.launch.py localization:=true
  ros2 launch rbpodo_slam rtabmap.launch.py delete_db:=false
  ros2 launch rbpodo_slam rtabmap.launch.py rtabmap_viz:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rtabmap_launch_path = PathJoinSubstitution([
        FindPackageShare('rtabmap_launch'), 'launch', 'rtabmap.launch.py'
    ])

    # Disable the wrapper's rtabmap_viz: upstream bug passes
    # wait_for_transform as string, rtabmap_viz expects double.
    # Spawn a separate rtabmap_viz node below with typed params.
    base_args = {
        'frame_id': 'camera_link',
        'rgb_topic': '/camera/camera/color/image_raw',
        'depth_topic': '/camera/camera/aligned_depth_to_color/image_raw',
        'camera_info_topic': '/camera/camera/color/camera_info',
        'approx_sync': 'true',
        'approx_sync_max_interval': '0.02',
        'qos': '2',
        'rgbd_sync': 'true',
        'visual_odometry': 'true',
        'publish_tf_odom': 'true',
        'odom_frame_id': 'odom',
        'rtabmap_viz': 'false',
        'rviz': LaunchConfiguration('rviz'),
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        'database_path': LaunchConfiguration('database_path'),
    }

    # Performance tuning flags shared by both modes:
    #   Vis/MaxFeatures    - fewer features = faster odom (500 vs default 1000)
    #   Rtabmap/TimeThr    - skip loop closure if over budget (ms)
    #   Mem/STMSize        - short-term memory nodes kept in RAM
    #   GFTT/MinDistance   - spread features evenly, reduce redundant work
    perf_args = (
        '--Rtabmap/DetectionRate 2.0'
        ' --Vis/MaxFeatures 500'
        ' --GFTT/MinDistance 10'
        ' --Rtabmap/TimeThr 700'
        ' --Mem/STMSize 10'
    )

    mapping_args = dict(base_args)
    mapping_args['rtabmap_args'] = PythonExpression([
        f"'{perf_args}' + (' --delete_db_on_start' if '",
        LaunchConfiguration('delete_db'), "'.lower() == 'true' else '')"
    ])

    localization_args = dict(base_args)
    localization_args['localization'] = 'true'
    localization_args['rtabmap_args'] = perf_args

    rtabmap_viz_node = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        namespace='rtabmap',
        output='screen',
        parameters=[{
            'frame_id': 'camera_link',
            'odom_frame_id': 'odom',
            'subscribe_rgbd': True,
            'subscribe_odom_info': True,
            'approx_sync': True,
            'wait_for_transform': 0.2,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        remappings=[
            ('rgbd_image', 'rgbd_image'),
            ('odom', 'odom'),
        ],
        condition=IfCondition(LaunchConfiguration('rtabmap_viz')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('localization', default_value='false',
                              description='Run RTAB-Map in localization mode (no mapping).'),
        DeclareLaunchArgument('delete_db', default_value='true',
                              description='Delete existing map database on start (mapping mode only).'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('rtabmap_viz', default_value='false',
                              description='Launch rtabmap_viz GUI (separate typed node, not the broken wrapper one).'),
        DeclareLaunchArgument('database_path', default_value='~/.ros/rtabmap.db'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rtabmap_launch_path),
            launch_arguments=mapping_args.items(),
            condition=UnlessCondition(LaunchConfiguration('localization')),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rtabmap_launch_path),
            launch_arguments=localization_args.items(),
            condition=IfCondition(LaunchConfiguration('localization')),
        ),
        rtabmap_viz_node,
    ])
