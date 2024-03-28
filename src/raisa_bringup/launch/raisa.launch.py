import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node

os.environ["RCUTILS_COLORIZED_OUTPUT"] = "1"
os.environ["RCUTILS_CONSOLE_OUTPUT_FORMAT"] = "[{severity}]: {message}"

path_config = PathJoinSubstitution(
    [get_package_share_directory("raisa_bringup"), "config"]
)


def generate_launch_description():
    io_stm32 = Node(
        package="raisa_io",
        executable="io_stm32",
        name="io_stm32",
        parameters=[
            {
                "ip": "192.168.50.2",
                "port": 9798,
            }
        ],
        output="screen",
        respawn=True,
    )

    io_vision = Node(
        package="pandu_ros2_kit",
        executable="io_vision",
        name="io_vision",
        parameters=[
            {
                "camera_path": "/dev/v4l/by-id/usb-046d_C922_Pro_Stream_Webcam_4C1A129F-video-index0",
                "camera_width": 320,
                "camera_height": 240,
                "camera_fps": 30,
                "output_width": 640,
                "output_height": 480,
            }
        ],
        output="screen",
        respawn=True,
    )

    # --------------------------------------------------------------------------

    imu_filter_madgwick_node = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        name="imu_filter_madgwick_node",
        parameters=[{"use_mag": False}],
        remappings=[("/imu/data_raw", "/imu_raw"), ("/imu/data", "/imu_filtered")],
        arguments=["--ros-args", "--log-level", "warn"],
        respawn=True,
    )

    realsense2_camera_node = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        name="realsense2_camera_node",
        parameters=[
            {
                "enable_accel": True,
                "enable_gyro": True,
                "unite_imu_method": 2,
                "align_depth.enable": True,
                "initial_reset": True,
                "depth_module.profile": "640x360x15",
                "rgb_camera.profile": "640x360x15",
            }
        ],
        remappings=[("/imu", "/imu_raw")],
        arguments=["--ros-args", "--log-level", "warn"],
        respawn=True,
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d",
            PathJoinSubstitution([path_config, "raisa.rviz"]),
            "--ros-args",
            "--log-level",
            "warn",
        ],
    )

    return LaunchDescription(
        [
            io_stm32,
            io_vision,
            imu_filter_madgwick_node,
            realsense2_camera_node,
            rviz2,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=rviz2, on_exit=[EmitEvent(event=Shutdown())]
                )
            ),
        ]
    )
