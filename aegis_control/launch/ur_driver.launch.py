from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription, LaunchContext
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    AndSubstitution,
    LaunchConfiguration,
    NotSubstitution,
    PathJoinSubstitution,
)


def generate_launch_description(context: LaunchContext) -> LaunchDescription:

    namespace = LaunchConfiguration("namespace")
    mock_hardware = LaunchConfiguration("mock_hardware")

    # TODO: Define common parameters for this UR launch file & UR's URDF in an external YAML file
    ur_type = "ur5e"
    robot_ip = "aegis_ur"

    use_tool_communication = "false"
    tool_device_name = "/tmp/ttyUR"

    controller_spawner_timeout = "10"  # s
    initial_joint_controller = "scaled_joint_trajectory_controller"
    initial_joint_controllers = PathJoinSubstitution(
        [FindPackageShare("aegis_description"), "config", "controllers.yaml"]
    )
    update_rate_config_file = PathJoinSubstitution(
        [FindPackageShare("aegis_description"), "config", ur_type, "update_rate.yaml"]
    )

    control_node = prepare_control_node(
        namespace, mock_hardware, initial_joint_controllers, update_rate_config_file
    )
    ur_control_node = prepare_ur_control_node(
        namespace, mock_hardware, initial_joint_controllers, update_rate_config_file
    )
    dashboard_client_node = prepare_dashboard_client_node(
        namespace, robot_ip, mock_hardware
    )
    tool_communication_node = prepare_tool_communication_node(
        namespace, robot_ip, use_tool_communication, tool_device_name
    )
    urscript_interface = prepare_urscript_interface(namespace, robot_ip)
    controller_stopper_node = prepare_controller_stopper_node(namespace, mock_hardware)

    def controller_spawner(controllers, active=True):
        inactive_flags = ["--inactive"] if not active else []
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "--controller-manager",
                "/controller_manager",
                "--controller-manager-timeout",
                controller_spawner_timeout,
            ]
            + inactive_flags
            + controllers,
        )

    controllers_active = [
        "joint_state_broadcaster",
        "io_and_status_controller",
        "speed_scaling_state_broadcaster",
        "force_torque_sensor_broadcaster",
        "tcp_pose_broadcaster",
        "ur_configuration_controller",
    ]
    controllers_inactive = [
        "scaled_joint_trajectory_controller",
        "joint_trajectory_controller",
        "forward_velocity_controller",
        "forward_position_controller",
        "passthrough_trajectory_controller",
    ]
    controllers_active.append(initial_joint_controller.perform(context))
    controllers_inactive.remove(initial_joint_controller.perform(context))

    controller_spawners = [controller_spawner(controllers_active)] + [
        controller_spawner(controllers_inactive, active=False)
    ]

    return LaunchDescription(
        [
            control_node,
            ur_control_node,
            dashboard_client_node,
            tool_communication_node,
            controller_stopper_node,
            urscript_interface,
        ]
        + controller_spawners
    )


def prepare_control_node(
    namespace: LaunchConfiguration,
    mock_hardware: LaunchConfiguration | str,
    initial_joint_controllers: LaunchConfiguration | str,
    update_rate_config_file: LaunchConfiguration | str,
) -> Node:
    return Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            update_rate_config_file,
            ParameterFile(initial_joint_controllers, allow_substs=True),
            {"namespace": namespace},
        ],
        output="screen",
        condition=IfCondition(mock_hardware),
    )


def prepare_controller_stopper_node(
    namespace: LaunchConfiguration,
    mock_hardware: LaunchConfiguration | str,
) -> Node:
    return Node(
        package="ur_robot_driver",
        executable="controller_stopper_node",
        name="controller_stopper",
        output="screen",
        emulate_tty=True,
        condition=UnlessCondition(mock_hardware),
        parameters=[
            {"headless_mode": "false"},
            {"joint_controller_active": "true"},
            {
                "consistent_controllers": [
                    "io_and_status_controller",
                    "force_torque_sensor_broadcaster",
                    "joint_state_broadcaster",
                    "speed_scaling_state_broadcaster",
                    "tcp_pose_broadcaster",
                    "ur_configuration_controller",
                ]
            },
            {"namespace", namespace},
        ],
    )


def prepare_urscript_interface(
    namespace: LaunchConfiguration, robot_ip: LaunchConfiguration | str
) -> Node:
    return Node(
        package="ur_robot_driver",
        executable="urscript_interface",
        parameters=[{"robot_ip": robot_ip, "namespace": namespace}],
        output="screen",
    )


def prepare_tool_communication_node(
    namespace: LaunchConfiguration,
    robot_ip: LaunchConfiguration | str,
    use_tool_communication: LaunchConfiguration | str,
    tool_device_name: LaunchConfiguration | str,
) -> Node:
    return Node(
        package="ur_robot_driver",
        condition=IfCondition(use_tool_communication),
        executable="tool_communication.py",
        name="ur_tool_comm",
        output="screen",
        parameters=[
            {
                "robot_ip": robot_ip,
                "device_name": tool_device_name,
                "namespace": namespace,
            }
        ],
    )


def prepare_dashboard_client_node(
    namespace: LaunchConfiguration,
    robot_ip: LaunchConfiguration | str,
    mock_hardware: LaunchConfiguration | str,
) -> Node:
    return Node(
        package="ur_robot_driver",
        condition=IfCondition(NotSubstitution(mock_hardware)),
        executable="dashboard_client",
        name="dashboard_client",
        output="screen",
        emulate_tty=True,
        parameters=[{"robot_ip": robot_ip}, {"namespace", namespace}],
    )


def prepare_ur_control_node(
    namespace: LaunchConfiguration,
    mock_hardware: LaunchConfiguration | str,
    initial_joint_controllers: LaunchConfiguration | str,
    update_rate_config_file: LaunchConfiguration | str,
) -> Node:
    return Node(
        package="ur_robot_driver",
        executable="ur_ros2_control_node",
        parameters=[
            update_rate_config_file,
            ParameterFile(initial_joint_controllers, allow_substs=True),
            {"namespace", namespace},
        ],
        output="screen",
        condition=UnlessCondition(mock_hardware),
    )
