# import json
# import yaml
# import tempfile
# from pathlib import Path
# from launch import LaunchDescription, LaunchContext
# from launch.actions import OpaqueFunction
# from launch.conditions import UnlessCondition
# from launch.substitutions import LaunchConfiguration
# from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes, Node
# from launch_ros.descriptions import ComposableNode
# from ament_index_python.packages import get_package_share_directory

# from launch import LaunchDescription

# # Bypassing the launch system to access local import
# import os
# import sys

# run_path = os.path.dirname(os.path.realpath(__file__))
# sys.path.append(run_path)
# from include.utils import controllers_spawner  # noqa E402


# def generate_launch_description() -> LaunchDescription:
#     return LaunchDescription([OpaqueFunction(function=launch_setup)])


# def launch_setup(context: LaunchContext) -> list[Node]:
#     # TODO(issue#22): Setup global log level configuration
#     log_level = "info"
#     if context.environment.get("GRIPPER_CAMERA_DEBUG") == "1":
#         log_level = "debug"

#     mock_hardware = LaunchConfiguration("mock_hardware", default="false")

#     params_camera_back = {
#         "camera_id": "camera_back",
#         "device_user_id": "basler1"
#     }

#     params_camera_front = {
#         "camera_id": "camera_front",
#         "device_user_id": "basler2"
#     }

#     return [
#         create_camera_node(
#             mock_hardware,
#             params_camera_back["camera_id"],
#             params_camera_back,
#             log_level,
#         ),
#         create_camera_node(
#             mock_hardware,
#             params_camera_front["camera_id"],
#             params_camera_front,
#             log_level,
#         ),
#         # create_rectify_node(cfg.mock_hardware, name_pro_scene),
#         # create_rectify_node(cfg.mock_hardware, name_pro_scene),
#     ]


# def create_camera_node(
#     mock_hardware: LaunchConfiguration,
#     name: str,
#     cam_params: dict,
#     log_level: str,
# ) -> LoadComposableNodes:
#     return ComposableNodeContainer(
#         condition=UnlessCondition(mock_hardware),
#         name=name + "_container",
#         namespace="",
#         package="rclcpp_components",
#         executable="component_container",
#         composable_node_descriptions=[
#             ComposableNode(
#                 package="pylon_ros2_camera_wrapper",
#                 plugin="pylon_ros2_camera.launch.py",
#                 name="pylon_ros2_camera.launch.py",
#                 parameters=[cam_params],
#             )
#         ],
#         arguments=["--ros-args", "--log-level", log_level],
#         output="both",
#     )


# # ros2 launch pylon_ros2_camera_wrapper pylon_ros2_camera.launch.py camera_id:=camera_back device_user_id:=basler1
# # ros2 launch pylon_ros2_camera_wrapper pylon_ros2_camera.launch.py camera_id:=camera_front device_user_id:=basler2

#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.launch_context import LaunchContext
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_node(context: LaunchContext):
    """Return the action to launch `pylon_ros2_camera_wrapper`.
    This is required to evaluate `respawn` as boolean.
    """

    # adapt if needed
    debug = False

    # launch configuration variables
    node_name_front = LaunchConfiguration("node_name_front")
    node_name_back = LaunchConfiguration("node_name_back")
    camera_id = LaunchConfiguration("camera_id")
    device_user_id_front = LaunchConfiguration("device_user_id_front")
    device_user_id_back = LaunchConfiguration("device_user_id_back")

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
    if debug:
        launch_prefix = ["xterm -e gdb -ex run --args"]
    else:
        launch_prefix = ""

    return [
        Node(
            package="pylon_ros2_camera_wrapper",
            namespace="",
            executable="pylon_ros2_camera_wrapper",
            name=node_name_front,
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
                    "device_user_id": device_user_id_front,
                },
            ],
        ),
        Node(
            package="pylon_ros2_camera_wrapper",
            namespace="",
            executable="pylon_ros2_camera_wrapper",
            name=node_name_back,
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
                    "device_user_id": device_user_id_back,
                },
            ],
        ),
    ]


def generate_launch_description():
    default_config_file = os.path.join(
        get_package_share_directory("aegis_control"),
        "config",
        "basler_cameras.yaml",
    )

    declare_node_name_front_cmd = DeclareLaunchArgument(
        "node_name_front",
        default_value="tool_camera_front",
        description="Name of the wrapper node.",
    )

    declare_node_name_back_cmd = DeclareLaunchArgument(
        "node_name_back",
        default_value="tool_camera_back",
        description="Name of the wrapper node.",
    )

    declare_camera_id_cmd = DeclareLaunchArgument(
        "camera_id",
        default_value="my_camera",
        description="Id of the camera. Used as node namespace.",
    )

    declare_device_user_id_front_cmd = DeclareLaunchArgument(
        "device_user_id_front",
        default_value="basler_front",
        description="Device user id of the camera.",
    )

    declare_device_user_id_back_cmd = DeclareLaunchArgument(
        "device_user_id_back",
        default_value="basler_back",
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

    # Define LaunchDescription variable and return it
    ld = LaunchDescription()

    # ld.add_action(declare_node_name_cmd)
    ld.add_action(declare_node_name_front_cmd)
    ld.add_action(declare_node_name_back_cmd)

    ld.add_action(declare_camera_id_cmd)
    
    # ld.add_action(declare_device_user_id_cmd)
    ld.add_action(declare_device_user_id_front_cmd)
    ld.add_action(declare_device_user_id_back_cmd)

    ld.add_action(declare_config_file_cmd)
    ld.add_action(declare_mtu_size_cmd)
    ld.add_action(declare_startup_user_set_cmd)
    ld.add_action(declare_enable_status_publisher_cmd)
    ld.add_action(declare_enable_current_params_publisher_cmd)

    ld.add_action(declare_respawn_cmd)

    ld.add_action(OpaqueFunction(function=_launch_node))

    return ld
