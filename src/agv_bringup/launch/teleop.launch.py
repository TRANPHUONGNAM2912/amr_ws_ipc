from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    max_speed_mps = LaunchConfiguration("max_speed_mps")
    publish_hz = LaunchConfiguration("publish_hz")
    wheel_separation = LaunchConfiguration("wheel_separation")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "cmd_vel_topic",
                default_value="/cmd_vel",
                description="Twist topic to publish",
            ),
            DeclareLaunchArgument(
                "max_speed_mps",
                default_value="1.0",
                description="Slider max speed in m/s (used as wheel speed for in-place turn)",
            ),
            DeclareLaunchArgument(
                "publish_hz",
                default_value="100.0",
                description="Publish rate while holding a button",
            ),
            DeclareLaunchArgument(
                "wheel_separation",
                default_value="0.38",
                description="Wheel separation L (m). Used to compute angular.z for in-place turns.",
            ),
            Node(
                package="agv_bringup",
                executable="teleop_panel.py",
                name="teleop_panel",
                output="screen",
                parameters=[
                    {
                        "cmd_vel_topic": cmd_vel_topic,
                        "max_speed_mps": max_speed_mps,
                        "publish_hz": publish_hz,
                        "wheel_separation": wheel_separation,
                    }
                ],
            ),
        ]
    )

