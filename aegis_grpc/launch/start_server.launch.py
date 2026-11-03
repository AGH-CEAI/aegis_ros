from launch import LaunchContext, LaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])


def launch_setup(context: LaunchContext) -> list[Node]:
    mock_hardware = (
        LaunchConfiguration("mock_hardware", default="false").perform(context).lower()
        == "true"
    )
    # MoveIt executes trajectories through this controller; AdmittanceService.Disable() restores it.
    # The scaled JTC does not work on mock hardware.
    fallback_controller = (
        "joint_trajectory_controller"
        if mock_hardware
        else "scaled_joint_trajectory_controller"
    )

    return [
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
                    "admittance_fallback_controller": fallback_controller,
                }
            ],
        )
    ]
