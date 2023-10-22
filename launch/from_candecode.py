from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

logger = LaunchConfiguration("log-level")

decode_config_file = PathJoinSubstitution(
    [FindPackageShare("ros2_candecode"), "config", "candecode.yaml"]
)

antares_config_file = PathJoinSubstitution(
    [FindPackageShare("ros2_diag2influxdb"), "config", "antares.yaml"]
)


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "log-level",
                default_value=["debug"],
                description="Logging level",
            ),
            Node(
                package="ros2_candecode",
                executable="candecode",
                name="candecode_node",
                output="log",
                parameters=[decode_config_file],
                arguments=['--ros-args', '--log-level', logger]
            ),
            Node(
                package="ros2_diag2influxdb",
                executable="diag2influxdb_node",
                name="diag2influxdb_node",
                output="log",
                parameters=[antares_config_file],
                arguments=['--ros-args', '--log-level', logger]
            )
        ]
    )
