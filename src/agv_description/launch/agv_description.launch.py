"""
Robot description module: load Xacro/URDF, run robot_state_publisher và
joint_state_publisher (zero position) để giữ đủ TF bánh xe cho RViz.

joint_state_publisher chỉ publish zero cho các revolute joint (bánh xe) khi
driver thật không publish /joint_states. Nếu driver có publish /joint_states
thì tắt node này bằng cách truyền publish_joint_states:=false.

Điều này KHÔNG ảnh hưởng SLAM vì Cartographer không dùng wheel TF.

Usage:
  ros2 launch agv_description agv_description.launch.py
  ros2 launch agv_description agv_description.launch.py publish_joint_states:=false
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    xacro_path = PathJoinSubstitution([
        FindPackageShare('agv_description'),
        'urdf',
        'b300_description.urdf.xacro',
    ])
    robot_description = ParameterValue(
        Command(['xacro', ' ', xacro_path]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='If true, subscribe to /clock (simulation).',
        ),
        DeclareLaunchArgument(
            'publish_joint_states',
            default_value='true',
            description=(
                'true  → chạy joint_state_publisher (zero) khi driver không publish /joint_states\n'
                'false → tắt, dùng khi driver thật đã publish /joint_states'
            ),
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        ),

        # Publish zero position cho các revolute joint (left_wheel, right_wheel)
        # để RViz hiển thị đủ model. Không ảnh hưởng SLAM.
        Node(
            condition=IfCondition(LaunchConfiguration('publish_joint_states')),
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'robot_description': robot_description,
            }],
        ),
    ])
