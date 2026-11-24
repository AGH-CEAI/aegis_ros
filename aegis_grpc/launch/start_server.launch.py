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
    # Contact tests against geometry that is in the MoveIt scene (cell, scene objects, octomap)
    # need this off: the reference legitimately goes up to max_offset "into" the surface.
    admittance_check_collisions = (
        LaunchConfiguration("admittance_check_collisions", default="true")
        .perform(context)
        .lower()
        == "true"
    )
    # Speed tests: 0.004 m/step = 0.1 m/s. The reference-to-measured lag grows with the speed, so a
    # faster motion also needs a larger max_offset_lin (which raises the contact force ceiling too)
    # and max_joint_step (rad per call; the UR5e joints top out at ~3.1 rad/s = 0.126 rad/call).
    admittance_max_step_lin = float(
        LaunchConfiguration("admittance_max_step_lin", default="0.004").perform(context)
    )
    admittance_max_offset_lin = float(
        LaunchConfiguration("admittance_max_offset_lin", default="0.02").perform(
            context
        )
    )
    admittance_max_joint_step = float(
        LaunchConfiguration("admittance_max_joint_step", default="0.08").perform(
            context
        )
    )
    # [m/s^2] of the TCP reference, 0 = off: without it a policy can jump from rest to full speed
    # within one call (0.4 m/s in 40 ms is 10 m/s^2 and ~16 N of tool inertia in the F/T).
    admittance_max_accel_lin = float(
        LaunchConfiguration("admittance_max_accel_lin", default="0.0").perform(context)
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
                    # Tuned for aegis_gym policy rate of 25 Hz: one StepTCP call per 40 ms.
                    # Per-call limits scale with the rate (0.004 m/step = 0.1 m/s).
                    "admittance_step_duration_s": 0.04,
                    "admittance_max_step_lin": admittance_max_step_lin,
                    "admittance_max_step_rot": 0.02,
                    "admittance_max_joint_step": admittance_max_joint_step,
                    # Force ceiling ~ K * max_offset = 500 N/m * 0.02 m = 10 N; keep >= 2x max_step_lin.
                    "admittance_max_offset_lin": admittance_max_offset_lin,
                    "admittance_max_accel_lin": admittance_max_accel_lin,
                    "admittance_check_collisions": admittance_check_collisions,
                }
            ],
        )
    ]
