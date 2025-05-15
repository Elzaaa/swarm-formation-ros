from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='swarm_control',
            executable='my_node',
            name='my_node',
            output='screen'
        )
    ])
