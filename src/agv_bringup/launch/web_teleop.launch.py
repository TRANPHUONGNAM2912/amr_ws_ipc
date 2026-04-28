from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    port = LaunchConfiguration("port")
    bind = LaunchConfiguration("bind")

    web_dir = get_package_share_directory("agv_bringup") + "/web"

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "port",
                default_value="8080",
                description="HTTP port for web teleop UI",
            ),
            DeclareLaunchArgument(
                "bind",
                default_value="0.0.0.0",
                description="Bind address for HTTP server (0.0.0.0 for LAN access)",
            ),
            # Humble does not expose XMLLaunchDescriptionSource; run rosbridge via ros2.
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "launch",
                    "rosbridge_server",
                    "rosbridge_websocket_launch.xml",
                ],
                output="screen",
            ),
            ExecuteProcess(
                cmd=["python3", "-m", "http.server", port, "--bind", bind],
                cwd=web_dir,
                output="screen",
            ),
        ]
    )

