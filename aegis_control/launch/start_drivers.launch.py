from launch import LaunchDescription, LaunchContext, LaunchDescriptionEntity
from launch.actions import IncludeLaunchDescription, OpaqueFunction

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare

# Bypassing the launch system to access local import
import os
import sys

run_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(run_path)
from include.utils import str2bool  # noqa E402


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([OpaqueFunction(function=launch_setup)])


def launch_setup(context: LaunchContext) -> list[LaunchDescriptionEntity]:
    launch_args = {
        "tf_prefix": LaunchConfiguration("tf_prefix", default=""),
        "mock_hardware": LaunchConfiguration("mock_hardware", default="false"),
    }
    mock_hardware_bool = str2bool(launch_args["mock_hardware"].perform(context))

    control_params_files = prepare_params_files()
    control_node = prepare_control_node(mock_hardware_bool, control_params_files)

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
        launch_arguments=launch_args.items(),
    )

    gripper_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("aegis_control"),
                    "launch",
                    "gripper_driver.launch.py",
                ]
            )
        ),
        launch_arguments=launch_args.items(),
    )

    return [
        ur_driver,
        control_node,
        gripper_driver,
        ft_sensor_driver,
        depthai_cameras_driver,
    ]


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
    gripper_cfg = ParameterFile(
        PathJoinSubstitution(
            [
                FindPackageShare("aegis_control"),
                "config",
                "controllers",
                "hande_gripper.yaml",
            ]
        ),
        allow_substs=True,
    )

    return [
        ur_controllers_cfg,
        ur_update_rate_config_file,
        ft_sensor_controllers_cfg,
        gripper_cfg,
    ]


def prepare_control_node(
    mock_hardware: bool, parameters: list[PathJoinSubstitution]
) -> Node:
    # The following is due to the design of the ur_robot_driver for ROS 2 Humble
    if mock_hardware:
        return prepare_fake_control_node(parameters)
    return prepare_real_control_node(parameters)


def prepare_fake_control_node(parameters: list[PathJoinSubstitution]) -> Node:
    return Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=parameters,
        remappings=[("~/robot_description", "robot_description")],
        output="screen",
    )


def prepare_real_control_node(parameters: list[PathJoinSubstitution]) -> Node:
    return Node(
        package="ur_robot_driver",
        executable="ur_ros2_control_node",
        parameters=parameters,
        remappings=[("~/robot_description", "robot_description")],
        output="screen",
    )
