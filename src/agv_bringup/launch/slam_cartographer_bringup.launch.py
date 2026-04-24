"""
Hardware + EKF + Cartographer SLAM (robot odometry + IMU + laser).

Không include `agv_description` ở đây — URDF + robot_state_publisher nằm trong
`agv_mapping/3_cartographer_odom.launch.py` (tham số use_robot_description).

Usage:
  ros2 launch agv_bringup slam_cartographer_bringup.launch.py
  ros2 launch agv_bringup slam_cartographer_bringup.launch.py use_sim_time:=true
  ros2 launch agv_bringup slam_cartographer_bringup.launch.py \\
    scan_topic:=/scan odom_topic:=/odometry/filtered imu_topic:=/imu/data
  ros2 launch agv_bringup slam_cartographer_bringup.launch.py rviz:=false
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
    mapping_share = get_package_share_directory("agv_mapping")

    imu_serial_port = LaunchConfiguration("imu_serial_port")
    imu_baudrate = LaunchConfiguration("imu_baudrate")
    imu_frame_id = LaunchConfiguration("imu_frame_id")
    imu_publish_rate = LaunchConfiguration("imu_publish_rate")

    base_serial_port = LaunchConfiguration("base_serial_port")
    base_baudrate = LaunchConfiguration("base_baudrate")
    base_params_file = LaunchConfiguration("base_params_file")

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_robot_description = LaunchConfiguration("use_robot_description")
    publish_joint_states = LaunchConfiguration("publish_joint_states")

    scan_topic = LaunchConfiguration("scan_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    imu_topic = LaunchConfiguration("imu_topic")
    rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    default_base_cfg = os.path.join(base_share, "config", "base_driver.yaml")
    default_rviz = os.path.join(
        get_package_share_directory("agv_description"), "rviz", "display.rviz"
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
                "use_robot_description",
                default_value="true",
                description="Forwarded to 3_cartographer_odom (URDF via agv_description there)",
            ),
            DeclareLaunchArgument(
                "publish_joint_states",
                default_value="true",
                description=(
                    "Forwarded to 3_cartographer_odom: false if base driver publishes /joint_states"
                ),
            ),
            DeclareLaunchArgument(
                "scan_topic",
                default_value="/scan",
                description="LaserScan topic for Cartographer",
            ),
            DeclareLaunchArgument(
                "odom_topic",
                default_value="/odometry/filtered",
                description="Odometry topic (EKF output); remap Cartographer odom input",
            ),
            DeclareLaunchArgument(
                "imu_topic",
                default_value="/imu/data",
                description="IMU topic for Cartographer; remap internal imu subscription",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Forwarded to 3_cartographer_odom: launch RViz2",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz,
                description="RViz2 config path (default: agv_description/rviz/display.rviz)",
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
                    os.path.join(mapping_share, "launch", "3_cartographer_odom.launch.py")
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "scan_topic": scan_topic,
                    "odom_topic": odom_topic,
                    "imu_topic": imu_topic,
                    "use_robot_description": use_robot_description,
                    "publish_joint_states": publish_joint_states,
                    "rviz": rviz,
                    "rviz_config": rviz_config,
                }.items(),
            ),
        ]
    )
