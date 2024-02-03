from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='base_pkg',
            executable='cmd_node',
            name='cmd_node'
        ),
        Node(
            package='base_pkg',
            executable='joy_node',
            name='joy_node'
        ),
    ])