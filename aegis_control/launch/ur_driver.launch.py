from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)


class URConfig:
    def __init__(self):
        # TODO: Define common parameters for this UR launch file & UR's URDF in an external YAML file
        self.ur_type = "ur5e"
        self.robot_ip = "aegis_ur"

        self.use_tool_communication = "false"
        self.tool_device_name = "/tmp/ttyUR"

        self.controller_spawner_timeout = "10"  # s
        self.initial_joint_controller = "scaled_joint_trajectory_controller"

        self.ur_controllers_cfg = PathJoinSubstitution(
            [FindPackageShare("aegis_description"), "config", "controllers.yaml"]
        )
        self.update_rate_config_file = PathJoinSubstitution(
            [
                FindPackageShare("aegis_description"),
                "config",
                "ur5e",
                "update_rate.yaml",
            ]
        )


def generate_launch_description() -> LaunchDescription:

    mock_hardware = LaunchConfiguration("mock_hardware")
    tf_prefix = DeclareLaunchArgument(
        "tf_prefix",
        default_value="",
        description="tf_prefix of the joint names, useful for "
        "multi-robot setup. If changed, also joint names in the controllers' configuration "
        "have to be updated.",
    )

    cfg = URConfig()

    control_node = prepare_control_node(mock_hardware, cfg)
    ur_control_node = prepare_ur_control_node(mock_hardware, cfg)
    dashboard_client_node = prepare_dashboard_client_node(mock_hardware, cfg)
    tool_communication_node = prepare_tool_communication_node(cfg)
    urscript_interface = prepare_urscript_interface(cfg)
    controller_stopper_node = prepare_controller_stopper_node(mock_hardware)

    def controller_spawner(controllers, active=True):
        inactive_flags = ["--inactive"] if not active else []
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "--controller-manager",
                "controller_manager",
                "--controller-manager-timeout",
                cfg.controller_spawner_timeout,
            ]
            + inactive_flags
            + controllers,
        )

    controllers_active = [
        "joint_state_broadcaster",
        "io_and_status_controller",
        "speed_scaling_state_broadcaster",
        "force_torque_sensor_broadcaster",
        "ur_configuration_controller",
    ]
    controllers_inactive = [
        "scaled_joint_trajectory_controller",
        "joint_trajectory_controller",
        "forward_velocity_controller",
        "forward_position_controller",
    ]
    controllers_active.append(cfg.initial_joint_controller)
    controllers_inactive.remove(cfg.initial_joint_controller)

    controller_spawners = [controller_spawner(controllers_active)] + [
        controller_spawner(controllers_inactive, active=False)
    ]

    return LaunchDescription(
        [
            tf_prefix,
            control_node,
            ur_control_node,
            dashboard_client_node,
            tool_communication_node,
            controller_stopper_node,
            urscript_interface,
        ]
        + controller_spawners
    )


def prepare_control_node(mock_hardware: LaunchConfiguration, cfg: URConfig) -> Node:
    return Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            cfg.update_rate_config_file,
            ParameterFile(cfg.ur_controllers_cfg, allow_substs=True),
        ],
        remappings=[("~/robot_description", "robot_description")],
        output="screen",
        condition=IfCondition(mock_hardware),
    )


def prepare_ur_control_node(mock_hardware: LaunchConfiguration, cfg: URConfig) -> Node:
    return Node(
        package="ur_robot_driver",
        executable="ur_ros2_control_node",
        parameters=[
            cfg.update_rate_config_file,
            ParameterFile(cfg.ur_controllers_cfg, allow_substs=True),
        ],
        remappings=[("~/robot_description", "robot_description")],
        output="screen",
        condition=UnlessCondition(mock_hardware),
    )


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
