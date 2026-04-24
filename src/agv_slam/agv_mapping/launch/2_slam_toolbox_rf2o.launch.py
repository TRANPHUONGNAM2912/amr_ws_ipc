from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    scan_topic = LaunchConfiguration("scan_topic")
    use_robot_description = LaunchConfiguration("use_robot_description")
    publish_joint_states = LaunchConfiguration("publish_joint_states")
    rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    rf2o_freq = LaunchConfiguration("rf2o_freq")
    rf2o_log_level = LaunchConfiguration("rf2o_log_level")

    default_params_file = (
        get_package_share_directory("agv_mapping")
        + "/config/slam_toolbox/mapper_async.yaml"
    )
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
                "params_file",
                default_value=default_params_file,
                description="slam_toolbox YAML config file",
            ),
            DeclareLaunchArgument(
                "scan_topic",
                default_value="/scan",
                description="LaserScan topic for rf2o and slam_toolbox",
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
                description="Start RViz2 with config",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz,
                description="RViz2 config path",
            ),
            DeclareLaunchArgument(
                "rf2o_freq",
                default_value="30.0",
                description="RF2O output frequency (should be >= lidar rate)",
            ),
            DeclareLaunchArgument(
                "rf2o_log_level",
                default_value="info",
                description="RF2O log level: debug|info|warn|error|fatal",
            ),

            Node(
                condition=UnlessCondition(use_robot_description),
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_footprint_to_base_link",
                arguments=["0", "0", "0", "0", "0", "0",
                           "base_footprint", "base_link"],
            ),
            Node(
                condition=UnlessCondition(use_robot_description),
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_link_to_laser_link",
                arguments=["0", "0", "0.185", "0", "0", "0",
                           "base_link", "laser_link"],
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

            TimerAction(
                period=1.0,
                actions=[
                    Node(
                        package="rf2o_laser_odometry",
                        executable="rf2o_laser_odometry_node",
                        name="rf2o_laser_odometry",
                        output="screen",
                        arguments=["--ros-args", "--log-level", rf2o_log_level],
                        parameters=[
                            {
                                "laser_scan_topic": scan_topic,
                                "odom_topic": "/odom_rf2o",
                                "publish_tf": True,
                                "base_frame_id": "base_footprint",
                                "odom_frame_id": "odom",
                                "init_pose_from_topic": "",
                                "freq": rf2o_freq,
                                "use_sim_time": use_sim_time,
                            }
                        ],
                    ),
                ],
            ),
            TimerAction(
                period=2.0,
                actions=[
                    Node(
                        package="slam_toolbox",
                        executable="async_slam_toolbox_node",
                        name="slam_toolbox",
                        output="screen",
                        parameters=[
                            params_file,
                            {"use_sim_time": use_sim_time, "scan_topic": scan_topic},
                        ],
                    ),
                ],
            ),
        ]
    )
