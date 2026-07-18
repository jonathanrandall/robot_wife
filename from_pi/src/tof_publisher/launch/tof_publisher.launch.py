from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="tof_publisher",
            executable="tof_publisher",
            name="tof_publisher",
            output="screen",
            emulate_tty=True,
        ),
    ])
