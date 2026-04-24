import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ===========================
    # 1️⃣ Launch Configs
    # ===========================
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    slam_params_file = LaunchConfiguration('slam_params_file')

    default_nav2_params = os.path.join(
        get_package_share_directory('agv_navigation'),
        'params',
        'nav2_params.yaml'
    )

    default_slam_params = os.path.join(
        get_package_share_directory('agv_navigation'),
        'params',
        'slam_toolbox_localization.yaml'
    )

    # ===========================
    # 2️⃣ Declare args
    # ===========================
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false'
    )

    declare_nav2_params = DeclareLaunchArgument(
        'params_file',
        default_value=default_nav2_params
    )

    declare_slam_params = DeclareLaunchArgument(
        'slam_params_file',
        default_value=default_slam_params
    )

    # ===========================
    # 3️⃣ SLAM Toolbox (LOCALIZATION)
    # ===========================
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time}, 
            slam_params_file
        ],
        arguments=['--ros-args', '--log-level', 'slam_toolbox:=debug']
    )

    # ===========================
    # 4️⃣ Nav2 core nodes
    # ===========================
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    # ===========================
    # 5️⃣ Lifecycle Manager
    # ===========================
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': [
                'controller_server',
                'planner_server',
                'bt_navigator',
                'behavior_server',
                'waypoint_follower'
            ]
        }],
    )
    # rviz_config = os.path.join(
    # get_package_share_directory('turtlebot3_navigation2'),
    # 'rviz',
    # 'tb3_navigation2.rviz'
    # )

    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     arguments=['-d', rviz_config],
    #     parameters=[{'use_sim_time': use_sim_time}],
    #     output='screen'
    # )

    # ===========================
    # 6️⃣ Launch
    # ===========================
    return LaunchDescription([
        declare_use_sim_time,
        declare_nav2_params,
        declare_slam_params,

        slam_toolbox,

        controller_server,
        planner_server,
        bt_navigator,
        behavior_server,
        waypoint_follower,

        lifecycle_manager,
        # rviz_node
    ])