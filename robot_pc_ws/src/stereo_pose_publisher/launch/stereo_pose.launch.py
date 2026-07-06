import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Running both models on the GPU simultaneously causes a MediaPipe tensor
    # write-contention error.  Default: pose on CPU, hand on GPU.
    use_gpu_pose_arg = DeclareLaunchArgument(
        'use_gpu_pose',
        default_value='false',  # both on CPU avoids tensor contention and is faster in practice
        description='Run the pose landmarker on the GPU (true) or CPU (false)',
    )
    use_gpu_hand_arg = DeclareLaunchArgument(
        'use_gpu_hand',
        default_value='false',
        description='Run the hand landmarker on the GPU (true) or CPU (false)',
    )

    node = Node(
        package='stereo_pose_publisher',
        executable='stereo_pose_node',
        name='stereo_pose_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'use_gpu_pose': LaunchConfiguration('use_gpu_pose'),
            'use_gpu_hand': LaunchConfiguration('use_gpu_hand'),
        }],
    )

    rviz_config = os.path.join(
        get_package_share_directory('jessica_description'), 'rviz', 'display.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([use_gpu_pose_arg, use_gpu_hand_arg, node, rviz_node])
