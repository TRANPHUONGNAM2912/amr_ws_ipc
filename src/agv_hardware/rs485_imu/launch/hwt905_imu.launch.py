#!/usr/bin/env python3
"""
Launch file for the HWT905-485 IMU node.

Usage:
    ros2 launch rs485_imu hwt905_imu.launch.py
    ros2 launch rs485_imu hwt905_imu.launch.py port:=/dev/ttyUSB1 baudrate:=921600

TF:
    Mặc định KHÔNG publish static TF base_link→imu_link, vì URDF của robot
    (agv_description) đã chứa joint imu_link_joint với đúng offset vật lý.
    Nếu bạn KHÔNG chạy robot_state_publisher (ví dụ test IMU riêng lẻ),
    bật static TF giả (xyz=0, rpy=0) bằng:
        ros2 launch rs485_imu hwt905_imu.launch.py publish_tf:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare('rs485_imu')
    default_params = PathJoinSubstitution([pkg_share, 'config', 'hwt905_params.yaml'])

    port_arg = DeclareLaunchArgument(
        'port', default_value='/dev/ttyUSB0',
        description='Serial port connected to the USB-RS485 adapter')

    baudrate_arg = DeclareLaunchArgument(
        'baudrate', default_value='230400',
        description='Baud rate (must match device setting)')

    frame_id_arg = DeclareLaunchArgument(
        'frame_id', default_value='imu_link',
        description='TF frame ID for the IMU sensor')

    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate', default_value='98.0',
        description='Publishing rate in Hz (≤ sensor bandwidth, currently 98 Hz)')

    publish_tf_arg = DeclareLaunchArgument(
        'publish_tf', default_value='false',
        description=(
            'Publish static TF base_link->imu_link with xyz=0/rpy=0. '
            'Để false khi URDF (agv_description) đã có imu_link_joint.'
        ))

    imu_node = Node(
        package='rs485_imu',
        executable='hwt905_imu_node',
        name='hwt905_imu_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            default_params,
            {
                'port':         LaunchConfiguration('port'),
                'baudrate':     LaunchConfiguration('baudrate'),
                'frame_id':     LaunchConfiguration('frame_id'),
                'publish_rate': LaunchConfiguration('publish_rate'),
            },
        ],
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_tf_publisher',
        condition=IfCondition(LaunchConfiguration('publish_tf')),
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_link',
            '--child-frame-id', LaunchConfiguration('frame_id'),
        ],
    )

    return LaunchDescription([
        port_arg,
        baudrate_arg,
        frame_id_arg,
        publish_rate_arg,
        publish_tf_arg,
        imu_node,
        static_tf,
    ])
