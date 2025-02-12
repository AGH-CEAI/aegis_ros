from launch import LaunchDescription, LaunchContext
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Bypassing the launch system to access local import
import os
import sys

run_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(run_path)
from include.utils import str2bool, controllers_spawner  # noqa E402


class URConfig:
    def __init__(self):
        # TODO(issue#11): Define common parameters for this UR launch file & UR's URDF in an external YAML file
        self.ur_type = "ur5e"
        self.robot_ip = "aegis_ur"

        self.use_tool_communication = "true"
        self.tool_device_name = "/tmp/ttyUR"

        self.real_initial_joint_controller = "scaled_joint_trajectory_controller"
        self.fake_initial_joint_controller = "joint_trajectory_controller"


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([OpaqueFunction(function=launch_setup)])


def launch_setup(context: LaunchContext) -> list[Node]:
    # tf_prefix is necessary to properly parse ur_controllers_cfg YAML file
    tf_prefix = LaunchConfiguration("tf_prefix")  # noqa F841
    mock_hardware = LaunchConfiguration("mock_hardware")
    mock_hardware_bool = str2bool(mock_hardware.perform(context))

    cfg = URConfig()

    dashboard_client_node = prepare_dashboard_client_node(mock_hardware, cfg)
    tool_communication_node = prepare_tool_communication_node(cfg)
    urscript_interface = prepare_urscript_interface(cfg)
    controller_stopper_node = prepare_controller_stopper_node(mock_hardware)

    controllers_active = [
        "joint_state_broadcaster",
        "io_and_status_controller",
        "speed_scaling_state_broadcaster",
        "ur_force_torque_sensor_broadcaster",
        # "tcp_pose_broadcaster", # TODO(issue#12): debug why this doesn't work
        "ur_configuration_controller",
    ]
    controllers_inactive = [
        "scaled_joint_trajectory_controller",
        "joint_trajectory_controller",
        "forward_velocity_controller",
        "forward_position_controller",
        # TODO(issue#12): debug why these controllers don't work
        # "force_mode_controller",
        # "passthrough_trajectory_controller",
        # "freedrive_mode_controller",
    ]

    init_joint_controller = (
        cfg.fake_initial_joint_controller
        if mock_hardware_bool
        else cfg.real_initial_joint_controller
    )
    controllers_active.append(init_joint_controller)
    controllers_inactive.remove(init_joint_controller)

    return [
        dashboard_client_node,
        tool_communication_node,
        controller_stopper_node,
        urscript_interface,
        controllers_spawner(controllers_active),
        controllers_spawner(controllers_inactive, active=False),
    ]


def prepare_dashboard_client_node(
    mock_hardware: LaunchConfiguration, cfg: URConfig
) -> Node:
    return Node(
        package="ur_robot_driver",
        condition=UnlessCondition(mock_hardware),
        executable="dashboard_client",
        name="dashboard_client",
        output="screen",
        emulate_tty=True,
        parameters=[{"robot_ip": cfg.robot_ip}],
    )


def prepare_tool_communication_node(cfg: URConfig) -> Node:
    return Node(
        package="ur_robot_driver",
        condition=IfCondition(cfg.use_tool_communication),
        executable="tool_communication.py",
        name="ur_tool_comm",
        output="screen",
        parameters=[
            {
                "robot_ip": cfg.robot_ip,
                "device_name": cfg.tool_device_name,
            }
        ],
    )


def prepare_urscript_interface(cfg: URConfig) -> Node:
    return Node(
        package="ur_robot_driver",
        executable="urscript_interface",
        parameters=[{"robot_ip": cfg.robot_ip}],
        output="screen",
    )


def prepare_controller_stopper_node(mock_hardware: LaunchConfiguration) -> Node:
    return Node(
        package="ur_robot_driver",
        executable="controller_stopper_node",
        name="controller_stopper",
        output="screen",
        emulate_tty=True,
        condition=UnlessCondition(mock_hardware),
        parameters=[
            {"headless_mode": False},
            {"joint_controller_active": True},
            {
                "consistent_controllers": [
                    "io_and_status_controller",
                    "force_torque_sensor_broadcaster",
                    "joint_state_broadcaster",
                    "speed_scaling_state_broadcaster",
                    "ur_configuration_controller",
                ]
            },
        ],
        remappings=[
            (
                "controller_manager/list_controllers",
                "~/controller_manager/list_controllers",
            )
        ],
    )
