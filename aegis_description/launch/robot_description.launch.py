from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.descriptions import ParameterValue


def generate_launch_description():

    tf_prefix = LaunchConfiguration("tf_prefix", default="")
    mock_hardware = LaunchConfiguration("mock_hardware", default="false")

    robot_description_str = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("aegis_description"),
                    "urdf",
                    "aegis.urdf.xacro",
                ]
            ),
            " ",
            "tf_prefix:=",
            tf_prefix,
            " ",
            "mock_hardware:=",
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
                    {
                        "robot_description": ParameterValue(
                            robot_description_str, value_type=str
                        )
                    }
                ],
            )
        ]
    )
