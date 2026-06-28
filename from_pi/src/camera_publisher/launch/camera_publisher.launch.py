from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="camera_publisher",
            executable="webcam_publisher",
            name="webcam_publisher",
            output="screen",
            emulate_tty=True,
        ),
    ])
