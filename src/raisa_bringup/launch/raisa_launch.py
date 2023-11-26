import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import RegisterEventHandler, EmitEvent, ExecuteProcess
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node


# Collection of parameters for raisa tested by Pandu Surya Tantra and Moh Ismarintan Zazuli.
# Should yo want to ask about these parameters, please contact them.
param_raisa = {'raisa.tf.camera': [0.00, 0.00, 0.00, 0.00, 0.00, 0.00],
               'raisa.tf.lidar': [0.20, 0.00, 0.13, 180.00, 0.00, 90.00],
               'raisa.conversion.encoder_odometry_pulse_to_meter': 0.000076586,
               'raisa.odometry.offset_x': 0.317696,
               'raisa.odometry.offset_y': 0.0}

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
                     'distance_min': 0.2,
                     'distance_max': 20.0}],
        remappings=[('/points_xyz', '/lidar/points_xyz'),
                    ('/points_xyzi', '/lidar/points_xyzi')])

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

    return LaunchDescription([
        imu_filter_madgwick_node,
        realsense2_camera_node,
        rviz2,
        io_lslidar_n301,
        io_stm32,
        pose_estimator,
        transform_broadcaster,
    ])
