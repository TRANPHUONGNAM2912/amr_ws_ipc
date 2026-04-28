"""
Hardware + EKF + slam_toolbox (robot odometry + IMU + laser).

Usage:
  ros2 launch agv_bringup slam_toolbox_bringup.launch.py
  ros2 launch agv_bringup slam_toolbox_bringup.launch.py use_sim_time:=true
  ros2 launch agv_bringup slam_toolbox_bringup.launch.py params_file:=/path/to/mapper_async.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    hins_share = get_package_share_directory("hins_le_ros2")
    imu_share = get_package_share_directory("rs485_imu")
    base_share = get_package_share_directory("rs485_main_ros2")
    ekf_share = get_package_share_directory("localization_robot")
    mapping_share = get_package_share_directory("agv_mapping")

    imu_serial_port = LaunchConfiguration("imu_serial_port")
    imu_baudrate = LaunchConfiguration("imu_baudrate")
    imu_frame_id = LaunchConfiguration("imu_frame_id")
    imu_publish_rate = LaunchConfiguration("imu_publish_rate")

    base_serial_port = LaunchConfiguration("base_serial_port")
    base_baudrate = LaunchConfiguration("base_baudrate")
    base_params_file = LaunchConfiguration("base_params_file")

    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    use_robot_description = LaunchConfiguration("use_robot_description")
    publish_joint_states = LaunchConfiguration("publish_joint_states")
    rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    default_base_cfg = os.path.join(base_share, "config", "base_driver.yaml")
    default_toolbox_params = os.path.join(
        mapping_share, "config", "slam_toolbox", "mapper_async.yaml"
    )
    default_rviz = os.path.join(
        get_package_share_directory("agv_description"), "rviz", "display.rviz"
    )
    desc_launch = os.path.join(
        get_package_share_directory("agv_description"), "launch", "agv_description.launch.py"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "imu_serial_port",
                default_value="/dev/ttyUSB0",
                description="Serial port for HWT905 IMU (USB-RS485 adapter)",
            ),
            DeclareLaunchArgument(
                "imu_baudrate",
                default_value="230400",
                description="IMU serial baud rate",
            ),
            DeclareLaunchArgument(
                "imu_frame_id",
                default_value="imu_link",
                description="TF frame_id published by the IMU node",
            ),
            DeclareLaunchArgument(
                "imu_publish_rate",
                default_value="98.0",
                description="IMU publish rate (Hz)",
            ),
            DeclareLaunchArgument(
                "base_serial_port",
                default_value="/dev/ttyS4",
                description="Serial port for RS485 base driver (wheel MCU)",
            ),
            DeclareLaunchArgument(
                "base_baudrate",
                default_value="115200",
                description="Base driver serial baud rate",
            ),
            DeclareLaunchArgument(
                "base_params_file",
                default_value=default_base_cfg,
                description="YAML parameters for base_driver_node",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Set true when playing a bag or using simulation clock",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=default_toolbox_params,
                description="slam_toolbox YAML config file",
            ),
            DeclareLaunchArgument(
                "use_robot_description",
                default_value="true",
                description=(
                    "true  → include agv_description (URDF TF)\n"
                    "false → minimal static TF base_footprint→base_link→laser_link"
                ),
            ),
            DeclareLaunchArgument(
                "publish_joint_states",
                default_value="true",
                description="Only if use_robot_description:=true (zero wheel joints for RViz)",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Launch RViz2",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz,
                description="RViz2 config path",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(hins_share, "launch", "hins_le_launch.py")
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(imu_share, "launch", "hwt905_imu.launch.py")
                ),
                launch_arguments={
                    "port": imu_serial_port,
                    "baudrate": imu_baudrate,
                    "frame_id": imu_frame_id,
                    "publish_rate": imu_publish_rate,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(base_share, "launch", "base_driver.launch.py")
                ),
                launch_arguments={
                    "serial_port": base_serial_port,
                    "baudrate": base_baudrate,
                    "params_file": base_params_file,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(ekf_share, "launch", "ekf.launch.py")
                )
            ),
            Node(
                condition=UnlessCondition(use_robot_description),
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_footprint_to_base_link",
                arguments=["0", "0", "0", "0", "0", "0", "base_footprint", "base_link"],
            ),
            Node(
                condition=UnlessCondition(use_robot_description),
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_link_to_laser_link",
                arguments=["0", "0", "0.185", "0", "0", "0", "base_link", "laser_link"],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(desc_launch),
                condition=IfCondition(use_robot_description),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "publish_joint_states": publish_joint_states,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(mapping_share, "launch", "1_slam_toolbox_odom.launch.py")
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "params_file": params_file,
                }.items(),
            ),
            Node(
                condition=IfCondition(rviz),
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
                arguments=["-d", rviz_config],
            ),
        ]
    )

