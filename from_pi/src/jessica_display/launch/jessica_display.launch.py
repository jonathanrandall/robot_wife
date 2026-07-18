from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="jessica_display",
            executable="display_node",
            name="jessica_display",
            output="screen",
            emulate_tty=True,
        ),
    ])
