import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    pkg_path = get_package_share_directory('jessica_description')
    default_xacro_path = os.path.join(pkg_path, 'description', 'jessica.urdf.xacro')
    default_rviz_path = os.path.join(pkg_path, 'rviz', 'display.rviz')

    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=default_xacro_path,
        description='Absolute path to the robot xacro/urdf file to load'
    )
    rvizconfig_arg = DeclareLaunchArgument(
        name='rvizconfig',
        default_value=default_rviz_path,
        description='Absolute path to the RViz config file'
    )
    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='true',
        description='Launch joint_state_publisher_gui to move non-fixed joints with sliders'
    )

    # use_ros2_control:=false since this is a display-only view: no controller_manager
    # is spawned, so there's no hardware/ESP32 backing the ros2_control interfaces.
    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model'), ' use_ros2_control:=false']),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui')),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')]
    )

    return LaunchDescription([
        model_arg,
        rvizconfig_arg,
        gui_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ])
