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
            io_vision,
            rviz2,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=rviz2, on_exit=[EmitEvent(event=Shutdown())]
                )
            ),
        ]
    )
