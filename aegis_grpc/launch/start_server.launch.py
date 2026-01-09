from launch_ros.actions import Node
from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="aegis_grpc",
                executable="grpc_server",
                output="both",
                parameters=[
                    {
                        "topic_pose": "/tcp_pose_broadcaster/pose",
                        "topic_wrench": "/net_ft_sensor_broadcaster/wrench",
                    }
                ],
            )
        ]
    )
