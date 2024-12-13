from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)


def generate_launch_description():

    namespace = LaunchConfiguration("namespace", default="")
    mock_hardware = LaunchConfiguration("mock_hardware", default="false")

    robot_description = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("aegis_control"),
                    "urdf",
                    "aegis_control.urdf.xacro",
                ]
            ),
            "use_mock_hardware:=",
            mock_hardware,
        ]
    )

    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="both",
                parameters=[
                    {"robot_description": robot_description},
                    {"namespace", namespace},
                ],
            )
        ]
    )
