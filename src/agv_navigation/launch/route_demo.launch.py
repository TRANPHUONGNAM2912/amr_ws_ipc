import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='agv_navigation',
            executable='route_demo.py',
            name='route_demo',
            output='screen',
            emulate_tty=True
        )
    ])
