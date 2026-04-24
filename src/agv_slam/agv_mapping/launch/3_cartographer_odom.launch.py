from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    scan_topic = LaunchConfiguration("scan_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    imu_topic = LaunchConfiguration("imu_topic")
    use_robot_description = LaunchConfiguration("use_robot_description")
    publish_joint_states = LaunchConfiguration("publish_joint_states")
    rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    config_dir = get_package_share_directory("agv_mapping") + "/config/cartographer"
    config_basename = "carto_with_odom.lua"
    desc_launch = (
        get_package_share_directory("agv_description")
        + "/launch/agv_description.launch.py"
    )
    default_rviz = (
        get_package_share_directory("agv_description") + "/rviz/display.rviz"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use /clock if true",
            ),
            DeclareLaunchArgument(
                "scan_topic",
                default_value="/scan",
                description="LaserScan topic for cartographer",
            ),
            DeclareLaunchArgument(
                "odom_topic",
                default_value="/odometry/filtered",
                description=(
                    "nav_msgs/Odometry for cartographer (internal name 'odom'). "
                    "Use /odom if you rely on raw wheel odometry instead of EKF."
                ),
            ),
            DeclareLaunchArgument(
                "imu_topic",
                default_value="/imu/data",
                description=(
                    "sensor_msgs/Imu for cartographer (internal name 'imu'). "
                    "HWT905 publishes here; default Cartographer expects /imu."
                ),
            ),
            DeclareLaunchArgument(
                "use_robot_description",
                default_value="true",
                description=(
                    "true  → include agv_description (robot_state_publisher + joint_state_publisher optional)\n"
                    "false → minimal static TF (same idea as 4_cartographer_no_odom)"
                ),
            ),
            DeclareLaunchArgument(
                "publish_joint_states",
                default_value="true",
                description=(
                    "Only when use_robot_description:=true: joint_state_publisher if driver has no /joint_states"
                ),
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Start RViz2 with rviz_config (same pattern as 4_cartographer_no_odom).",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz,
                description="RViz2 config file path.",
            ),
            Node(
                condition=UnlessCondition(use_robot_description),
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_footprint_to_base_link",
                arguments=[
                    "0", "0", "0", "0", "0", "0",
                    "base_footprint", "base_link",
                ],
            ),
            Node(
                condition=UnlessCondition(use_robot_description),
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_link_to_laser_link",
                arguments=[
                    "0", "0", "0.185", "0", "0", "0",
                    "base_link", "laser_link",
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(desc_launch),
                condition=IfCondition(use_robot_description),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "publish_joint_states": publish_joint_states,
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
            Node(
                package="cartographer_ros",
                executable="cartographer_node",
                name="cartographer_node",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
                arguments=[
                    "-configuration_directory",
                    config_dir,
                    "-configuration_basename",
                    config_basename,
                ],
                remappings=[
                    ("scan", scan_topic),
                    ("odom", odom_topic),
                    ("imu", imu_topic),
                ],
            ),
            Node(
                package="cartographer_ros",
                executable="cartographer_occupancy_grid_node",
                name="cartographer_occupancy_grid_node",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time, "resolution": 0.05}],
            ),
        ]
    )
