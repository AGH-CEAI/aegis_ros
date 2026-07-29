import os
import time
from pathlib import Path

import numpy as np
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import UnlessCondition
from launch.launch_context import LaunchContext
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from rclpy.logging import get_logger
from scipy.spatial.transform import Rotation


def generate_launch_description():
    return LaunchDescription(
        declare_launch_arguments()
        + [
            OpaqueFunction(function=launch_nodes),
        ]
    )


def launch_nodes(context: LaunchContext):
    """Return the action to launch `pylon_ros2_camera_wrapper`.
    This is required to evaluate `respawn` as boolean.
    """

    # adapt if needed
    debug = False

    mock_hardware = LaunchConfiguration("mock_hardware", default="false")

    # launch configuration variables
    node_name_left = LaunchConfiguration("node_name_left")
    node_name_right = LaunchConfiguration("node_name_right")
    node_name_left_str = node_name_left.perform(context)
    node_name_right_str = node_name_right.perform(context)

    device_user_id_left = LaunchConfiguration("device_user_id_left")
    device_user_id_right = LaunchConfiguration("device_user_id_right")

    config_file = LaunchConfiguration("config_file")
    roi_file = LaunchConfiguration("roi_file")

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

    logger = get_logger("pylon_cameras_driver")

    calibration_extrinsics_paths = {
        node_name_right_str: Path(
            "~/ceai_ws/src/aegis_ros/aegis_utils/config/tool_right_extrinsics.yaml"
        ).expanduser(),
        node_name_left_str: Path(
            "~/ceai_ws/src/aegis_ros/aegis_utils/config/tool_left_extrinsics.yaml"
        ).expanduser(),
    }

    files_missing = {
        name: not path.is_file() for name, path in calibration_extrinsics_paths.items()
    }

    if any(files_missing.values()):
        logger.warning(
            "Following calibration files are missing:\n"
            + "\n".join(
                str(calibration_extrinsics_paths[name])
                for name, missing in files_missing.items()
                if missing
            )
        )
        logger.warning(
            "[WARN] Static TF will not be published for cameras:\n"
            + "\n".join(name for name, missing in files_missing.items() if missing)
        )
        time.sleep(6)

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

    rectify_tool_node = create_rectify_node(
        cam_tool_left_str=node_name_left_str, cam_tool_right_str=node_name_right_str
    )
    static_tf_tool_right_node = create_static_tf_node(
        files_missing,
        calibration_extrinsics_paths,
        "robotiq_hande_end",
        node_name_right_str,
    )
    static_tf_tool_left_node = create_static_tf_node(
        files_missing,
        calibration_extrinsics_paths,
        "robotiq_hande_end",
        node_name_left_str,
    )

    roi_setter_node = create_roi_setter_node(
        config_file=roi_file, camera_names=f"{node_name_left_str},{node_name_right_str}"
    )

    white_balance_setter_node = create_white_balance_setter_node(
        config_file=config_file,
        camera_names=f"{node_name_left_str},{node_name_right_str}",
    )

    return [
        node_camera_left,
        node_camera_right,
        rectify_tool_node,
        static_tf_tool_right_node,
        static_tf_tool_left_node,
        roi_setter_node,
        white_balance_setter_node,
    ]


# TODO(issue#55) Add rectifying nodes
def create_rectify_node(
    cam_tool_left_str: str, cam_tool_right_str: str
) -> LoadComposableNodes:
    return ComposableNodeContainer(
        name="cam_tool_rectify_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="image_proc",
                plugin="image_proc::DebayerNode",
                name="cam_tool_left_debayer_node",
                namespace=cam_tool_left_str,
            ),
            ComposableNode(
                package="image_proc",
                plugin="image_proc::DebayerNode",
                name="cam_tool_right_debayer_node",
                namespace=cam_tool_right_str,
            ),
        ],
        arguments=["--ros-args", "--log-level", "info"],
        output="both",
    )


def create_static_tf_node(
    files_missing: dict[str, bool],
    calibration_extrinsics_paths: dict[str, Path],
    parent_frame: str,
    child_frame: str,
) -> Node:
    if not files_missing[child_frame]:
        with open(calibration_extrinsics_paths[child_frame], "r") as f:
            data = yaml.safe_load(f)
        T = np.array(data["T_base2cam" if "scene" in child_frame else "T_tcp2cam"])
        x, y, z = T[:3, 3]
        qx, qy, qz, qw = Rotation.from_matrix(T[:3, :3]).as_quat()
    else:
        x, y, z, qx, qy, qz, qw = 0, 0, 0, 0, 0, 0, 1

    return Node(
        condition=UnlessCondition(
            TextSubstitution(text=str(files_missing[child_frame]).lower())
        ),
        package="tf2_ros",
        executable="static_transform_publisher",
        name=f"static_tf_{child_frame}",
        arguments=[
            str(x),
            str(y),
            str(z),
            str(qx),
            str(qy),
            str(qz),
            str(qw),
            parent_frame,
            f"{child_frame}_camera_optical_frame_cal",
        ],
    )


def create_roi_setter_node(
    config_file: LaunchConfiguration, camera_names: LaunchConfiguration
) -> Node:
    return Node(
        package="aegis_control",
        executable="pylon_roi_setter.py",
        name="pylon_roi_setter",
        output="screen",
        parameters=[
            {
                "config_file": config_file,
                "camera_names": camera_names,
            }
        ],
    )


def create_white_balance_setter_node(
    config_file: LaunchConfiguration, camera_names: LaunchConfiguration
) -> Node:
    return Node(
        package="aegis_control",
        executable="pylon_white_balance_setter.py",
        name="pylon_white_balance_setter",
        output="screen",
        parameters=[
            {
                "config_file": config_file,
                "camera_names": camera_names,
            }
        ],
    )


def declare_launch_arguments() -> list:
    default_config_file = PathJoinSubstitution(
        [FindPackageShare("aegis_control"), "config", "cameras", "pylon_cameras.yaml"]
    )
    default_roi_file = PathJoinSubstitution(
        [FindPackageShare("aegis_control"), "config", "cameras", "roi.yaml"]
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

    declare_roi_file_cmd = DeclareLaunchArgument(
        "roi_file",
        default_value=default_roi_file,
        description="Cameras ROI in a .yaml file.",
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

    return [
        declare_node_name_left_cmd,
        declare_node_name_right_cmd,
        declare_camera_id_cmd,
        declare_device_user_id_left_cmd,
        declare_device_user_id_right_cmd,
        declare_config_file_cmd,
        declare_roi_file_cmd,
        declare_mtu_size_cmd,
        declare_startup_user_set_cmd,
        declare_enable_status_publisher_cmd,
        declare_enable_current_params_publisher_cmd,
        declare_respawn_cmd,
    ]
