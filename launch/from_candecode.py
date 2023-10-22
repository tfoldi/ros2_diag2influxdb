from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


decode_config_file = PathJoinSubstitution(
    [FindPackageShare("ros2_candecode"), "config", "candecode.yaml"]
)

antares_config_file = PathJoinSubstitution(
    [FindPackageShare("ros2_diag2influxdb"), "config", "antares.yaml"]
)


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="ros2_candecode",
                executable="candecode",
                name="candecode_node",
                output="log",
                parameters=[decode_config_file],
                arguments=['--ros-args', '--log-level', 'debug']
            ),
            Node(
                package="ros2_diag2influxdb",
                executable="diag2influxdb_node",
                name="diag2influxdb_node",
                output="log",
                parameters=[antares_config_file],
                arguments=['--ros-args', '--log-level', 'debug']
            )
        ]
    )
