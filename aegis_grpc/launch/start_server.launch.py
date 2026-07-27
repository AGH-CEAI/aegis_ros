from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="aegis_grpc",
                executable="grpc_server",
                output="both",
                parameters=[
                    {
                        "tcp_frame": "robotiq_hande_end",
                        "base_frame": "world",
                        "topic_wrench": "/net_ft_sensor_broadcaster/wrench",
                        "topics_history_depth": 1,
                    }
                ],
            )
        ]
    )
