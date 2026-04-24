"""
Full hardware + localization + Nav2 stack for real robot navigation.

Starts (in dependency order for readability; all processes run in parallel):
  1. Robot URDF + robot_state_publisher (agv_description)
  2. Hins LiDAR (hins_le_ros2)
  3. HWT905 IMU + static TF (rs485_imu)
  4. RS485 base driver / wheel odometry (rs485_main_ros2)
  5. EKF (robot_localization, localization_robot)
  6. Nav2 with static map (agv_navigation)

Usage:
  ros2 launch agv_bringup navigation_bringup.launch.py
  ros2 launch agv_bringup navigation_bringup.launch.py map:=/path/to/map.yaml use_rviz:=false
  ros2 launch agv_bringup navigation_bringup.launch.py \\
    imu_serial_port:=/dev/ttyUSB0 base_serial_port:=/dev/ttyS4
  ros2 launch agv_bringup navigation_bringup.launch.py publish_joint_states:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    hins_share = get_package_share_directory("hins_le_ros2")
    imu_share = get_package_share_directory("rs485_imu")
    base_share = get_package_share_directory("rs485_main_ros2")
    ekf_share = get_package_share_directory("localization_robot")
    nav_share = get_package_share_directory("agv_navigation")
    desc_share = get_package_share_directory("agv_description")

    imu_serial_port = LaunchConfiguration("imu_serial_port")
    imu_baudrate = LaunchConfiguration("imu_baudrate")
    imu_frame_id = LaunchConfiguration("imu_frame_id")
    imu_publish_rate = LaunchConfiguration("imu_publish_rate")

    base_serial_port = LaunchConfiguration("base_serial_port")
    base_baudrate = LaunchConfiguration("base_baudrate")
    base_params_file = LaunchConfiguration("base_params_file")

    use_sim_time = LaunchConfiguration("use_sim_time")
    publish_joint_states = LaunchConfiguration("publish_joint_states")
    autostart = LaunchConfiguration("autostart")
    map_yaml = LaunchConfiguration("map")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    default_map = os.path.join(nav_share, "maps", "warehouse_map_20_04.yaml")
    default_base_cfg = os.path.join(base_share, "config", "base_driver.yaml")
    default_rviz = os.path.join(nav_share, "rviz", "nav2_default_view.rviz")

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
                "publish_joint_states",
                default_value="true",
                description=(
                    "Forwarded to agv_description: false if base driver publishes /joint_states"
                ),
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="true",
                description="Nav2 lifecycle_manager autostart",
            ),
            DeclareLaunchArgument(
                "map",
                default_value=default_map,
                description="Path to map_server YAML (with sibling .pgm)",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Launch RViz2 with Nav2 config",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz,
                description="RViz2 config file path",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(desc_share, "launch", "agv_description.launch.py")
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "publish_joint_states": publish_joint_states,
                }.items(),
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
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav_share, "launch", "nav2_with_static_map.launch.py")
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
#                    "map": map_yaml,
                    "use_rviz": use_rviz,
                    "rviz_config": rviz_config,
                }.items(),
            ),
        ]
    )
