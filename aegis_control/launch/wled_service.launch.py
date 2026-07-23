from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:

    config_file_arg = DeclareLaunchArgument(
        "config",
        default_value=PathJoinSubstitution(
            [FindPackageShare("aegis_control"), "config/wled", "scenes.yaml"]
        ),
        description="Path to the custom YAML configuration file for WLED service.",
    )

    mock_hardware_arg = DeclareLaunchArgument(
        "mock_hardware",
        default_value="false",
        description="Flag to indicate whether to run in mock hardware mode.",
    )

    config_file = LaunchConfiguration("config")
    mock_hardware_val = LaunchConfiguration("mock_hardware")

    wled_service_node = Node(
        package="wled_ros_driver",
        executable="start_wled_service",
        name="wled_service_node",
        output="screen",
        parameters=[config_file, {"mock_hardware": mock_hardware_val}],
    )

    return LaunchDescription(
        [
            config_file_arg,
            mock_hardware_arg,
            wled_service_node,
        ]
    )
