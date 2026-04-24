"""
Visualize the URDF offline: includes the minimal description module, then adds
joint_state_publisher_gui (for dummy wheel joints) and RViz2 with display.rviz.

RViz is loaded with Fixed Frame base_footprint and a RobotModel on /robot_description.
Opening RViz without a config often leaves Fixed Frame = map (from an old default) and
no RobotModel display — that is why the model did not appear.

Do not use this on the real vehicle if the MCU publishes /joint_states — use
agv_description.launch.py from bringup instead.

Usage: ros2 launch agv_description display.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('agv_description')
    desc_launch = os.path.join(pkg_share, 'launch', 'agv_description.launch.py')
    default_rviz = os.path.join(pkg_share, 'rviz', 'display.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Must match RViz; false for bench visualization',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=default_rviz,
            description='Path to RViz2 config (Fixed Frame + RobotModel)',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(desc_launch),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }.items(),
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            arguments=['-d', LaunchConfiguration('rviz_config')],
        ),
    ])
