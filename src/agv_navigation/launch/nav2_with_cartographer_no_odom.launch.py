from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav_pkg = get_package_share_directory("agv_navigation")
    slam_pkg = get_package_share_directory("agv_mapping")

    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    scan_topic = LaunchConfiguration("scan_topic")

    amcl_file = nav_pkg + "/config/localization/amcl.yaml"
    controller_file = nav_pkg + "/config/controller/controller_server.yaml"
    local_costmap_file = nav_pkg + "/config/costmap/local_costmap.yaml"
    global_costmap_file = nav_pkg + "/config/costmap/global_costmap.yaml"
    nav2_core_file = nav_pkg + "/config/common/nav2_core.yaml"
    profile_file = nav_pkg + "/config/profiles/b300_sim.yaml"

    params_files = [
        amcl_file,
        controller_file,
        local_costmap_file,
        global_costmap_file,
        nav2_core_file,
        profile_file,
    ]

    lifecycle_nodes = [
        "controller_server",
        "planner_server",
        "smoother_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
        "velocity_smoother",
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time", default_value="false", description="Use /clock if true"
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="true",
                description="Automatically transition Nav2 lifecycle nodes",
            ),
            DeclareLaunchArgument(
                "use_rviz", default_value="true", description="Launch RViz2"
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=nav_pkg + "/rviz/nav2_default_view.rviz",
                description="RViz config path",
            ),
            DeclareLaunchArgument(
                "scan_topic",
                default_value="/scan",
                description="LaserScan topic for Cartographer and costmaps",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    slam_pkg + "/launch/4_cartographer_no_odom.launch.py"
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "scan_topic": scan_topic,
                    "rviz": "false",
                    "use_robot_description": "true",
                }.items(),
            ),
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                output="screen",
                parameters=params_files + [{"use_sim_time": use_sim_time}],
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                parameters=params_files + [{"use_sim_time": use_sim_time}],
            ),
            Node(
                package="nav2_smoother",
                executable="smoother_server",
                name="smoother_server",
                output="screen",
                parameters=params_files + [{"use_sim_time": use_sim_time}],
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                output="screen",
                parameters=params_files + [{"use_sim_time": use_sim_time}],
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                parameters=params_files + [{"use_sim_time": use_sim_time}],
            ),
            Node(
                package="nav2_waypoint_follower",
                executable="waypoint_follower",
                name="waypoint_follower",
                output="screen",
                parameters=params_files + [{"use_sim_time": use_sim_time}],
            ),
            Node(
                package="nav2_velocity_smoother",
                executable="velocity_smoother",
                name="velocity_smoother",
                output="screen",
                parameters=params_files + [{"use_sim_time": use_sim_time}],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "autostart": autostart,
                        "node_names": lifecycle_nodes,
                    }
                ],
            ),
            Node(
                condition=IfCondition(use_rviz),
                package="rviz2",
                executable="rviz2",
                name="rviz2_nav",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
                arguments=["-d", rviz_config],
            ),
        ]
    )
