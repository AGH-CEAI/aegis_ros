from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    launch_args = {
        "tf_prefix": LaunchConfiguration("tf_prefix", default=""),
        "mock_hardware": LaunchConfiguration("mock_hardware", default="false"),
    }

    control_params_files = prepare_params_files()
    control_nodes = prepare_control_nodes(
        launch_args["mock_hardware"], control_params_files
    )

    ur_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("aegis_control"), "launch", "ur_driver.launch.py"]
            )
        ),
        launch_arguments=launch_args.items(),
    )

    ft_sensor_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("aegis_control"),
                    "launch",
                    "ft_sensor_driver.launch.py",
                ]
            )
        ),
        launch_arguments=launch_args.items(),
    )

    depthai_cameras_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("aegis_control"),
                    "launch",
                    "depthai_cameras_driver.launch.py",
                ]
            )
        ),
    )

    return LaunchDescription(
        [
            ur_driver,
            ft_sensor_driver,
            depthai_cameras_driver,
        ]
        + control_nodes
    )


def prepare_params_files() -> list[PathJoinSubstitution]:
    ur_controllers_cfg = ParameterFile(
        PathJoinSubstitution(
            [
                FindPackageShare("aegis_control"),
                "config",
                "controllers",
                "ur_drivers.yaml",
            ]
        ),
        allow_substs=True,
    )
    ur_update_rate_config_file = PathJoinSubstitution(
        [
            FindPackageShare("aegis_control"),
            "config",
            "controllers",
            "update_rate.yaml",
        ]
    )
    ft_sensor_controllers_cfg = PathJoinSubstitution(
        [
            FindPackageShare("aegis_control"),
            "config",
            "controllers",
            "net_ft_broadcaster.yaml",
        ]
    )

    return [
        ur_controllers_cfg,
        ur_update_rate_config_file,
        ft_sensor_controllers_cfg,
    ]


def prepare_control_nodes(
    mock_hardware: LaunchConfiguration, parameters: list[PathJoinSubstitution]
) -> list[Node]:
    # The following is due to the design of the ur_robot_driver for ROS 2 Humble
    return [
        prepare_fake_control_node(mock_hardware, parameters),
        prepare_real_control_node(mock_hardware, parameters),
    ]


def prepare_fake_control_node(
    mock_hardware: LaunchConfiguration, parameters: list[PathJoinSubstitution]
) -> Node:
    return Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=parameters,
        remappings=[("~/robot_description", "robot_description")],
        output="screen",
        condition=IfCondition(mock_hardware),
    )


def prepare_real_control_node(
    mock_hardware: LaunchConfiguration, parameters: list[PathJoinSubstitution]
) -> Node:
    return Node(
        package="ur_robot_driver",
        executable="ur_ros2_control_node",
        parameters=parameters,
        remappings=[("~/robot_description", "robot_description")],
        output="screen",
        condition=UnlessCondition(mock_hardware),
    )
