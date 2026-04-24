#!/usr/bin/env python3
"""
Launch file for the RS485 base driver node.

Usage:
  ros2 launch rs485_main_ros2 base_driver.launch.py
  ros2 launch rs485_main_ros2 base_driver.launch.py serial_port:=/dev/ttyUSB0
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share    = FindPackageShare('rs485_main_ros2')
    default_cfg  = PathJoinSubstitution([pkg_share, 'config', 'base_driver.yaml'])

    port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyS4',
        description='RS485 serial port device path',
    )
    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='115200',
        description='Serial baud rate',
    )
    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_cfg,
        description='Full path to the ROS2 parameters YAML file',
    )

    base_driver_node = Node(
        package='rs485_main_ros2',
        executable='base_driver_node',
        name='base_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'serial_port': LaunchConfiguration('serial_port'),
                'baudrate':    LaunchConfiguration('baudrate'),
            },
        ],
    )

    return LaunchDescription([
        port_arg,
        baudrate_arg,
        params_arg,
        base_driver_node,
    ])
