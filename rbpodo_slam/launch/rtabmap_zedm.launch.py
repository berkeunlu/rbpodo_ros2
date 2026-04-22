"""RTAB-Map RGBD SLAM launch for ZED Mini on rbpodo rb10_1300e.

Assumes the ZED wrapper is already running:
  ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedm camera_name:=zedm

odom_source options:
  rtabmap (default):
    Rtabmap runs its own visual odometry.
    frame_id = link0 (robot base).
    TF chain: map -> odom -> link0 -> ... -> camera_mount_link -> zedm_camera_link
    robot_state_publisher (FK) handles link0 -> ... -> zedm_camera_link.

  zed:
    Rtabmap uses ZED's internal VIO (IMU-fused) as external odometry.
    frame_id = zedm_left_camera_frame (camera frame).
    TF chain: map -> odom -> zedm_camera_link -> zedm_left_camera_frame
    ZED wrapper publishes the odom -> zedm_camera_link TF.
    NOTE: robot links are NOT part of the map TF tree in this mode.

Example use:
  ros2 launch rbpodo_slam rtabmap_zedm.launch.py
  ros2 launch rbpodo_slam rtabmap_zedm.launch.py odom_source:=zed
  ros2 launch rbpodo_slam rtabmap_zedm.launch.py localization:=true
  ros2 launch rbpodo_slam rtabmap_zedm.launch.py delete_db:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):

    zed_ns       = context.launch_configurations['zed_namespace']
    odom_source  = context.launch_configurations['odom_source']
    localization = context.launch_configurations['localization']
    delete_db    = context.launch_configurations['delete_db']
    use_sim_time = context.launch_configurations['use_sim_time']
    database_path = context.launch_configurations['database_path']
    rviz         = context.launch_configurations['rviz']
    rtabmap_viz_flag = context.launch_configurations['rtabmap_viz']

    rtabmap_launch_path = PathJoinSubstitution([
        FindPackageShare('rtabmap_launch'), 'launch', 'rtabmap.launch.py'
    ]).perform(context)

    rgb_topic   = f'/{zed_ns}/zed_node/rgb/color/rect/image'
    depth_topic = f'/{zed_ns}/zed_node/depth/depth_registered'
    camera_info = f'/{zed_ns}/zed_node/rgb/color/rect/camera_info'
    odom_topic  = f'/{zed_ns}/zed_node/odom'

    perf_args = (
        '--Rtabmap/DetectionRate 2.0'
        ' --Vis/MaxFeatures 500'
        ' --GFTT/MinDistance 10'
        ' --Rtabmap/TimeThr 700'
        ' --Mem/STMSize 10'
    )

    if odom_source == 'zed':
        # ZED VIO as external odom. Camera frame is the tracked body.
        # TF: map -> odom -> zedm_camera_link (ZED publishes this)
        base_args = {
            'frame_id': f'{zed_ns}_left_camera_frame',
            'odom_topic': odom_topic,
            'visual_odometry': 'false',
            'publish_tf_odom': 'false',
        }
    else:
        # Rtabmap visual odometry. Robot base (link0) is the tracked body.
        # TF: map -> odom -> link0 -> ... -> zedm_camera_link (via FK)
        base_args = {
            'frame_id': 'link0',
            'visual_odometry': 'true',
            'publish_tf_odom': 'true',
        }

    base_args.update({
        'rgb_topic': rgb_topic,
        'depth_topic': depth_topic,
        'camera_info_topic': camera_info,
        'approx_sync': 'true',
        'approx_sync_max_interval': '0.05',
        'qos': '1',
        'odom_frame_id': 'odom',
        'rtabmap_viz': 'false',
        'rviz': rviz,
        'use_sim_time': use_sim_time,
        'database_path': database_path,
        'rgbd_sync': 'true',
    })

    mapping_args = dict(base_args)
    mapping_args['rtabmap_args'] = (
        perf_args + (' --delete_db_on_start' if delete_db.lower() == 'true' else '')
    )

    localization_args = dict(base_args)
    localization_args['localization'] = 'true'
    localization_args['rtabmap_args'] = perf_args

    viz_frame = f'{zed_ns}_left_camera_frame' if odom_source == 'zed' else 'link0'

    nodes = []

    if localization.lower() == 'true':
        nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rtabmap_launch_path),
            launch_arguments=localization_args.items(),
        ))
    else:
        nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rtabmap_launch_path),
            launch_arguments=mapping_args.items(),
        ))

    if rtabmap_viz_flag.lower() == 'true':
        nodes.append(Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmap_viz',
            namespace='rtabmap',
            output='screen',
            parameters=[{
                'frame_id': viz_frame,
                'odom_frame_id': 'odom',
                'subscribe_rgbd': True,
                'subscribe_odom_info': True,
                'approx_sync': True,
                'wait_for_transform': 0.2,
                'use_sim_time': use_sim_time == 'true',
            }],
            remappings=[
                ('rgbd_image', 'rgbd_image'),
                ('odom', 'odom'),
            ],
        ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('zed_namespace', default_value='zedm',
                              description='ZED camera_name used when launching zed_wrapper.'),
        DeclareLaunchArgument('odom_source', default_value='rtabmap',
                              description='Odometry source: "rtabmap" (visual VO, frame=link0) '
                                          'or "zed" (ZED VIO, frame=zedm_left_camera_frame).'),
        DeclareLaunchArgument('localization', default_value='false',
                              description='Localization mode (no new mapping).'),
        DeclareLaunchArgument('delete_db', default_value='true',
                              description='Delete map database on start (mapping mode only).'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('rtabmap_viz', default_value='false'),
        DeclareLaunchArgument('database_path', default_value='~/.ros/rtabmap_zedm.db'),

        OpaqueFunction(function=launch_setup),
    ])
