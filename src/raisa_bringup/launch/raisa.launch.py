import os

from launch import LaunchDescription
from launch_ros.actions import Node

os.environ["RCUTILS_COLORIZED_OUTPUT"] = "1"
os.environ["RCUTILS_CONSOLE_OUTPUT_FORMAT"] = "[{severity}]: {message}"


def generate_launch_description():
    io_vision = Node(
        package="pandu_ros2_kit",
        executable="io_vision",
        name="io_vision",
        parameters=[
            {
                "camera_path": "/dev/v4l/by-id/usb-SunplusIT_Inc_HD_Webcam-video-index0",
                "camera_width": 640,
                "camera_height": 360,
                "camera_fps": 30,
                "output_width": 640,
                "output_height": 360,
                "output_fps": 30,
            }
        ],
        output="screen",
        respawn=True,
    )

    return LaunchDescription([io_vision])
