from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='target_localization',
            executable='target_localization',
            name='target_localization',
            output='screen'
        )
    ])
