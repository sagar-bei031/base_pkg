from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='base_pkg',
            executable='esp_node',
            name='esp'
        ),
        Node(
            package='base_pkg',
            executable='turtlesim_node',
            name='filter'
        ),
    ])