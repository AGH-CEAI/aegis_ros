from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="aegis_director",
                executable="director",
                name="director",
                output="screen",
            )
        ]
    )
