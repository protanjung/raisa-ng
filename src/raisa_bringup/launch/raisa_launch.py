import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import EmitEvent, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node


# Collection of parameters for raisa tested by Pandu Surya Tantra and Moh Ismarintan Zazuli.
# Should yo want to ask about these parameters, please contact them.
param_raisa = {'raisa.tf.camera': [0.24, 0.00, 0.30, 0.00, 0.00, 0.00],
               'raisa.tf.lidar': [0.20, 0.00, 0.35, 180.00, 0.00, 90.00],
               'raisa.conversion.odometry_pulse_to_meter': 0.000076586,
               'raisa.conversion.roda_pulse_to_meter': 0.0000820008,
               'raisa.conversion.stm32_from_pc_linear_multiplier': 170.0,
               'raisa.conversion.stm32_from_pc_angular_multiplier': 28.0,
               'raisa.odometry.offset_x': 0.317696,
               'raisa.odometry.offset_y': 0.0,
               'raisa.roda.offset_x': -0.05,
               'raisa.roda.offset_y': 0.0}

# if 'GTK_PATH' environment variable is set, rviz2 will crash
# to avoid this, delete the variable before launching rviz2
if 'GTK_PATH' in os.environ:
    del os.environ['GTK_PATH']

# set rcutils logging format to only show severity and message
# also force colorized output to be enabled
os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = "[{severity}]: {message}"
os.environ['RCUTILS_COLORIZED_OUTPUT'] = "1"

raisa_ng_data_path = os.path.join(os.environ['HOME'], 'raisa-ng-data')


def generate_launch_description():
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        respawn=True,
        parameters=[{'map_frame': 'map',
                     'odom_frame': 'odom',
                     'base_link_frame': 'base_link',
                     'world_frame': 'map',
                     'two_d_mode': True,
                     'smooth_lagged_data': True,
                     'history_length': 10.0,
                     'odom0': '/odom',
                     'odom0_config': [True, True, False,
                                      False, False, True,
                                      False, False, False,
                                      False, False, False,
                                      False, False, False],
                     'odom0_differential': True,
                     'odom0_relative': True,
                     'pose0': '/slam/localization_pose',
                     'pose0_config': [True, True, False,
                                      False, False, True,
                                      False, False, False,
                                      False, False, False,
                                      False, False, False],
                     'pose0_differential': False,
                     'pose0_relative': False}])

    imu_filter_madgwick_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick_node',
        respawn=True,
        parameters=[{'use_mag': False}],
        remappings=[('/imu/data_raw', '/imu_raw'),
                    ('/imu/data', '/imu_filtered')])

    realsense2_camera_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='realsense2_camera_node',
        respawn=True,
        parameters=[{'enable_accel': True,
                     'enable_gyro': True,
                     'unite_imu_method': 2,
                     'align_depth.enable': True,
                     'initial_reset': True}],
        remappings=[('/imu', '/imu_raw')])

    rtabmap_slam_rtabmap = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        namespace='slam',
        respawn=True,
        parameters=[{'approx_sync': True,
                     'subscribe_depth': False,
                     'subscribe_scan': True,
                     'subscribe_scan_cloud': False,
                     'subscribe_stereo': False,
                     'subscribe_rgbd': True,
                     'subscribe_rgb': False,
                     'odom_frame_id': 'odom',
                     'odom_tf_linear_variance': 0.001,
                     'odom_tf_angular_variance': 0.001,
                     'publish_tf': False,
                     'Grid/CellSize': '0.02',
                     'Grid/FootprintHeight': '1.5',
                     'Grid/FootprintLength': '0.5',
                     'Grid/FootprintWidth': '0.5',
                     'Grid/FromDepth': 'False',
                     'Mem/IncrementalMemory': 'True',
                     'RGBD/NeighborLinkRefining': 'True',
                     'RGBD/ProximityBySpace': 'True',
                     'RGBD/AngularUpdate': '0.01',
                     'RGBD/LinearUpdate': '0.01',
                     'RGBD/OptimizeFromGraphEnd': 'False',
                     'Reg/Force3DoF': 'true',
                     'Reg/Strategy': '1'}],
        remappings=[('odom', '/odom'),
                    ('scan', '/scan')],
        arguments=['--delete_db_on_start'])

    rtabmap_sync_rgbd_sync = Node(
        package='rtabmap_sync',
        executable='rgbd_sync',
        name='rgbd_sync',
        namespace='slam',
        respawn=True,
        parameters=[{'approx_sync': False}],
        remappings=[('rgb/image', '/color/image_raw'),
                    ('rgb/camera_info', '/color/camera_info'),
                    ('depth/image', '/aligned_depth_to_color/image_raw')])

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([raisa_ng_data_path, 'rviz', 'raisa.rviz'])]
    )

    io_lslidar_n301 = Node(
        package='raisa_io',
        executable='io_lslidar_n301',
        name='io_lslidar_n301',
        respawn=True,
        parameters=[{'msop_port': 2368,
                     'difop_port': 2369,
                     'frame_id': 'lidar_link',
                     'azimuth_start': 185.0,
                     'azimuth_stop': 355.0,
                     'azimuth_step': 1.0,
                     'distance_min': 0.2,
                     'distance_max': 20.0}],
        remappings=[('/points_xyz', '/lidar/points_xyz'),
                    ('/points_xyzi', '/lidar/points_xyzi'),
                    ('/laser_scan', '/scan')])

    io_stm32 = Node(
        package='raisa_io',
        executable='io_stm32',
        name='io_stm32',
        respawn=True,
        parameters=[{'stm32.ip': '192.168.50.2',
                     'stm32.port': 9798}])

    pose_estimator = Node(
        package='raisa_middleware',
        executable='pose_estimator',
        name='pose_estimator',
        respawn=True,
        parameters=[param_raisa])

    transform_broadcaster = Node(
        package='raisa_middleware',
        executable='transform_broadcaster',
        name='transform_broadcaster',
        respawn=True,
        parameters=[param_raisa])

    routine = Node(
        package='raisa_routine',
        executable='routine',
        name='routine',
        respawn=True,
        parameters=[param_raisa])

    return LaunchDescription([
        ekf_node,
        imu_filter_madgwick_node,
        realsense2_camera_node,
        rtabmap_slam_rtabmap,
        rtabmap_sync_rgbd_sync,
        rviz2,
        io_lslidar_n301,
        io_stm32,
        pose_estimator,
        transform_broadcaster,
        routine,
    ])
