from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("agv_navigation")
    # Map YAML (and sibling .pgm) are installed to share/agv_navigation/maps/
    default_map_yaml = pkg_share + "/maps/warehouse_map_20_04.yaml"

    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    map_yaml = LaunchConfiguration("map")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    nav2_params_file = pkg_share + "/config/nav2_params.yaml"

    params_files = [
        nav2_params_file,
    ]

    lifecycle_nodes = [
        "map_server",
        "amcl",
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
                "map",
                default_value=default_map_yaml,
                description=(
                    "Path to map yaml for map_server. Default: "
                    "agv_navigation/share/maps/map_warehouse_20_04.yaml "
                    "(override with map:=/absolute/path/to/map.yaml)"
                ),
            ),
            DeclareLaunchArgument(
                "use_rviz", default_value="true", description="Launch RViz2"
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=pkg_share + "/rviz/nav2_default_view.rviz",
                description="RViz config path",
            ),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=params_files
                + [{"use_sim_time": use_sim_time, "yaml_filename": map_yaml}],
            ),
            Node(
                package="nav2_amcl",
                executable="amcl",
                name="amcl",
                output="screen",
                parameters=params_files + [{"use_sim_time": use_sim_time}],
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
                name="rviz2",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
                arguments=["-d", rviz_config],
            ),
        ]
    )
