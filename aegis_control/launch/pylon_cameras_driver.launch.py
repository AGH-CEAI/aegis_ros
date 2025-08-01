#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.launch_context import LaunchContext
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition
from launch_ros.actions import Node


def generate_launch_description():
    default_config_file = os.path.join(
        get_package_share_directory("aegis_control"),
        "config",
        "cameras",
        "pylon_cameras.yaml",
    )

    declare_node_name_left_cmd = DeclareLaunchArgument(
        "node_name_left",
        default_value="cam_tool_left",
        description="Name of the wrapper node.",
    )

    declare_node_name_right_cmd = DeclareLaunchArgument(
        "node_name_right",
        default_value="cam_tool_right",
        description="Name of the wrapper node.",
    )

    declare_camera_id_cmd = DeclareLaunchArgument(
        "camera_id",
        default_value="my_camera",
        description="Id of the camera. Used as node namespace.",
    )

    declare_device_user_id_left_cmd = DeclareLaunchArgument(
        "device_user_id_left",
        default_value="basler_left",
        description="Device user id of the camera.",
    )

    declare_device_user_id_right_cmd = DeclareLaunchArgument(
        "device_user_id_right",
        default_value="basler_right",
        description="Device user id of the camera.",
    )

    declare_config_file_cmd = DeclareLaunchArgument(
        "config_file",
        default_value=default_config_file,
        description="Camera parameters structured in a .yaml file.",
    )

    declare_mtu_size_cmd = DeclareLaunchArgument(
        "mtu_size",
        default_value="1500",
        description="Maximum transfer unit size. To enable jumbo frames, set it to a high value (8192 recommended)",
    )

    declare_startup_user_set_cmd = DeclareLaunchArgument(
        "startup_user_set",
        # possible value: Default, UserSet1, UserSet2, UserSet3, CurrentSetting
        default_value="CurrentSetting",
        description="Specific user set defining user parameters to run the camera.",
    )

    declare_enable_status_publisher_cmd = DeclareLaunchArgument(
        "enable_status_publisher",
        default_value="true",
        description="Enable/Disable the status publishing.",
    )

    declare_enable_current_params_publisher_cmd = DeclareLaunchArgument(
        "enable_current_params_publisher",
        default_value="true",
        description="Enable/Disable the current parameter publishing.",
    )

    declare_respawn_cmd = DeclareLaunchArgument(
        "respawn",
        default_value="false",
        description="If true, the node will be respawned if it exits.",
    )

    return LaunchDescription(
        [
            # declare_node_name_cmd,
            declare_node_name_left_cmd,
            declare_node_name_right_cmd,
            declare_camera_id_cmd,
            # declare_device_user_id_cmd.
            declare_device_user_id_left_cmd,
            declare_device_user_id_right_cmd,
            declare_config_file_cmd,
            declare_mtu_size_cmd,
            declare_startup_user_set_cmd,
            declare_enable_status_publisher_cmd,
            declare_enable_current_params_publisher_cmd,
            declare_respawn_cmd,
            OpaqueFunction(function=launch_node),
        ]
    )


def launch_node(context: LaunchContext):
    """Return the action to launch `pylon_ros2_camera_wrapper`.
    This is required to evaluate `respawn` as boolean.
    """

    # adapt if needed
    debug = False

    mock_hardware = LaunchConfiguration("mock_hardware", default="false")

    # launch configuration variables
    node_name_left = LaunchConfiguration("node_name_left")
    node_name_right = LaunchConfiguration("node_name_right")
    device_user_id_left = LaunchConfiguration("device_user_id_left")
    device_user_id_right = LaunchConfiguration("device_user_id_right")

    config_file = LaunchConfiguration("config_file")

    mtu_size = LaunchConfiguration("mtu_size")
    startup_user_set = LaunchConfiguration("startup_user_set")
    enable_status_publisher = LaunchConfiguration("enable_status_publisher")
    enable_current_params_publisher = LaunchConfiguration(
        "enable_current_params_publisher"
    )

    respawn = LaunchConfiguration("respawn")
    respawn_str = respawn.perform(context)
    respawn_bool = respawn_str.lower() == "true"

    # log format
    os.environ["RCUTILS_CONSOLE_OUTPUT_FORMAT"] = (
        "{time} [{name}] [{severity}] {message}"
    )

    # see https://navigation.ros.org/tutorials/docs/get_backtrace.html
    launch_prefix = ["xterm -e gdb -ex run --args"] if debug else ""

    node_camera_left = Node(
        package="pylon_ros2_camera_wrapper",
        namespace="",
        executable="pylon_ros2_camera_wrapper",
        condition=UnlessCondition(mock_hardware),
        name=node_name_left,
        output="screen",
        respawn=respawn_bool,
        emulate_tty=True,
        prefix=launch_prefix,
        parameters=[
            config_file,
            {
                "gige/mtu_size": mtu_size,
                "startup_user_set": startup_user_set,
                "enable_status_publisher": enable_status_publisher,
                "enable_current_params_publisher": enable_current_params_publisher,
                "device_user_id": device_user_id_left,
            },
        ],
    )

    node_camera_right = Node(
        package="pylon_ros2_camera_wrapper",
        namespace="",
        executable="pylon_ros2_camera_wrapper",
        condition=UnlessCondition(mock_hardware),
        name=node_name_right,
        output="screen",
        respawn=respawn_bool,
        emulate_tty=True,
        prefix=launch_prefix,
        parameters=[
            config_file,
            {
                "gige/mtu_size": mtu_size,
                "startup_user_set": startup_user_set,
                "enable_status_publisher": enable_status_publisher,
                "enable_current_params_publisher": enable_current_params_publisher,
                "device_user_id": device_user_id_right,
            },
        ],
    )

    return [
        node_camera_left,
        node_camera_right,
    ]
