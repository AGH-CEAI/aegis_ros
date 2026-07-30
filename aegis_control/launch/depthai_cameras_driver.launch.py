import json
import tempfile
import time
from pathlib import Path

import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import OpaqueFunction
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode
from rclpy.logging import get_logger
from scipy.spatial.transform import Rotation


# TODO(issue#107) Make DepthAI camera pipeline configurable
class DepthAIConfig:
    def __init__(self):
        self._modify_config(enable_nn=False)

        # TODO(issue#26) Introduce a mock for the DepthAI cameras
        self.mock_hardware = LaunchConfiguration("mock_hardware", default="false")

        self.cam_scene_name = LaunchConfiguration("cam_scene_name", default="cam_scene")
        self.cam_scene_model = LaunchConfiguration(
            "cam_scene_model", default="OAK-D-S2"
        )
        self.cam_scene_parent_frame = LaunchConfiguration(
            "cam_scene_parent_frame", default="cell"
        )
        self.cam_scene_base_frame = LaunchConfiguration(
            "cam_scene_base_frame", default="cam_scene_frame"
        )
        self.cam_scene_x = LaunchConfiguration("cam_scene_x", default="0.014")
        self.cam_scene_y = LaunchConfiguration("cam_scene_y", default="0.33")
        self.cam_scene_z = LaunchConfiguration("cam_scene_z", default="1.972")
        self.cam_scene_roll = LaunchConfiguration("cam_scene_roll", default="1.5708")
        self.cam_scene_pitch = LaunchConfiguration("cam_scene_pitch", default="1.5708")
        self.cam_scene_yaw = LaunchConfiguration("cam_scene_yaw", default="0")

        self.cam_tool_name = LaunchConfiguration(
            "cam_tool_name", default="cam_tool_front"
        )
        self.cam_tool_model = LaunchConfiguration(
            "cam_tool_model", default="OAK-D-SR-POE"
        )
        self.cam_tool_parent_frame = LaunchConfiguration(
            "cam_tool_parent_frame", default="camera_tool"
        )
        self.cam_tool_base_frame = LaunchConfiguration(
            "cam_tool_base_frame", default="cam_tool_frame"
        )
        self.cam_tool_x = LaunchConfiguration("cam_tool_x", default="0.003")
        self.cam_tool_y = LaunchConfiguration("cam_tool_y", default="-0.019")
        self.cam_tool_z = LaunchConfiguration("cam_tool_z", default="-0.039")
        self.cam_tool_roll = LaunchConfiguration("cam_tool_roll", default="0")
        self.cam_tool_pitch = LaunchConfiguration("cam_tool_pitch", default="1.5701")
        self.cam_tool_yaw = LaunchConfiguration("cam_tool_yaw", default="0")

    def _modify_config(self, enable_nn: bool) -> None:
        # TODO(issue#31) Fix YOLO configuration not being applied correctly
        package_share_path = Path(get_package_share_directory("aegis_control"))
        cam_src_params_path = (
            package_share_path / "config" / "cameras" / "depthai_cameras.yaml"
        )

        if not enable_nn:
            self.cam_params_path = cam_src_params_path
            return

        model_path = Path.home() / "ceai_models" / "yolo.blob"
        yolo_src_cfg_path = package_share_path / "config" / "cameras" / "yolo.json"

        self.yolo_cfg_path = Path(
            tempfile.NamedTemporaryFile(suffix=".json", delete=False).name
        )
        self.cam_params_path = Path(
            tempfile.NamedTemporaryFile(suffix=".yaml", delete=False).name
        )

        with open(yolo_src_cfg_path, "r") as file:
            yolo_cfg = json.load(file)

        yolo_cfg["model"]["model_name"] = str(model_path)

        with open(self.yolo_cfg_path, "w") as file:
            json.dump(yolo_cfg, file, indent=2)

        with open(cam_src_params_path, "r") as file:
            cam_params = yaml.safe_load(file)

        cam_params["/cam_scene"]["ros__parameters"]["nn"]["i_nn_config_path"] = str(
            self.yolo_cfg_path
        )

        with open(self.cam_params_path, "w") as file:
            yaml.safe_dump(cam_params, file)


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([OpaqueFunction(function=launch_setup)])


def launch_setup(context: LaunchContext) -> list[Node]:
    # TODO(issue#22): Setup global log level configuration
    log_level = "info"
    if context.environment.get("DEPTHAI_DEBUG") == "1":
        log_level = "debug"

    cfg = DepthAIConfig()
    cam_scene_name = cfg.cam_scene_name.perform(context)
    cam_tool_name = cfg.cam_tool_name.perform(context)

    # TODO(issue#23): Investigate the necessity of tf parameters
    tf_params_cam_scene = {
        "camera": {
            "i_publish_tf_from_calibration": False,
            "i_tf_tf_prefix": cam_scene_name,
            "i_tf_camera_model": cfg.cam_scene_model,
            "i_tf_parent_frame": cfg.cam_scene_parent_frame.perform(context),
            "i_tf_base_frame": cfg.cam_scene_base_frame.perform(context),
            "i_tf_cam_pos_x": cfg.cam_scene_x.perform(context),
            "i_tf_cam_pos_y": cfg.cam_scene_y.perform(context),
            "i_tf_cam_pos_z": cfg.cam_scene_z.perform(context),
            "i_tf_cam_roll": cfg.cam_scene_roll.perform(context),
            "i_tf_cam_pitch": cfg.cam_scene_pitch.perform(context),
            "i_tf_cam_yaw": cfg.cam_scene_yaw.perform(context),
        }
    }

    tf_params_cam_tool = {
        "camera": {
            "i_publish_tf_from_calibration": False,
            "i_tf_tf_prefix": cam_tool_name,
            "i_tf_camera_model": cfg.cam_tool_model,
            "i_tf_parent_frame": cfg.cam_tool_parent_frame.perform(context),
            "i_tf_base_frame": cfg.cam_tool_base_frame.perform(context),
            "i_tf_cam_pos_x": cfg.cam_tool_x.perform(context),
            "i_tf_cam_pos_y": cfg.cam_tool_y.perform(context),
            "i_tf_cam_pos_z": cfg.cam_tool_z.perform(context),
            "i_tf_cam_roll": cfg.cam_tool_roll.perform(context),
            "i_tf_cam_pitch": cfg.cam_tool_pitch.perform(context),
            "i_tf_cam_yaw": cfg.cam_tool_yaw.perform(context),
        }
    }

    logger = get_logger("depthai_cameras_driver")

    calibration_extrinsics_paths = {
        "cam_scene_rgb": Path(
            "~/ceai_ws/src/aegis_ros/aegis_utils/config/scene_extrinsics.yaml"
        ).expanduser(),
        "cam_tool_front_right": Path(
            "~/ceai_ws/src/aegis_ros/aegis_utils/config/tool_front_right_extrinsics.yaml"
        ).expanduser(),
        "cam_tool_front_left": Path(
            "~/ceai_ws/src/aegis_ros/aegis_utils/config/tool_front_left_extrinsics.yaml"
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

    camera_scene_node = create_camera_node(
        cfg.mock_hardware,
        cam_scene_name,
        tf_params_cam_scene,
        cfg.cam_params_path,
        log_level,
    )
    camera_tool_node = create_camera_node(
        cfg.mock_hardware,
        cam_tool_name,
        tf_params_cam_tool,
        cfg.cam_params_path,
        log_level,
    )
    rectify_scene_node = create_rectify_node(cfg.mock_hardware, cam_scene_name, "/rgb")
    rectify_tool_right_node = create_rectify_node(
        cfg.mock_hardware, cam_tool_name, "/right"
    )
    rectify_tool_left_node = create_rectify_node(
        cfg.mock_hardware, cam_tool_name, "/left"
    )
    # spatial_bb_node = create_spatial_bb_node(
    #     cfg.mock_hardware, cam_scene_name, cfg.cam_params_path
    # )
    # point_cloud_node = create_point_cloud_node(cfg.mock_hardware, cam_scene_name)
    static_tf_scene_node = create_static_tf_node(
        files_missing,
        calibration_extrinsics_paths,
        "base_link",
        "cam_scene_rgb",
    )
    static_tf_tool_front_right_node = create_static_tf_node(
        files_missing,
        calibration_extrinsics_paths,
        "robotiq_hande_end",
        "cam_tool_front_right",
    )
    static_tf_tool_front_left_node = create_static_tf_node(
        files_missing,
        calibration_extrinsics_paths,
        "robotiq_hande_end",
        "cam_tool_front_left",
    )

    # TODO(issue#107) Make DepthAI camera pipeline configurable
    return [
        camera_scene_node,
        camera_tool_node,
        rectify_scene_node,
        rectify_tool_right_node,
        rectify_tool_left_node,
        # spatial_bb_node,
        # point_cloud_node,
        static_tf_scene_node,
        static_tf_tool_front_right_node,
        static_tf_tool_front_left_node,
    ]


def create_camera_node(
    mock_hardware: LaunchConfiguration,
    name: str,
    tf_params: dict,
    cam_params_path: LaunchConfiguration,
    log_level: str,
) -> LoadComposableNodes:
    return ComposableNodeContainer(
        condition=UnlessCondition(mock_hardware),
        name=name + "_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="depthai_ros_driver",
                plugin="depthai_ros_driver::Camera",
                name=name,
                parameters=[cam_params_path, tf_params],
                extra_arguments=[{"use_intra_process_comms": True}],
            )
        ],
        arguments=["--ros-args", "--log-level", log_level],
        output="both",
    )


def create_rectify_node(
    mock_hardware: LaunchConfiguration, name: str, type: str
) -> LoadComposableNodes:
    side = "" if type == "/rgb" else "_" + type[1:]
    return LoadComposableNodes(
        condition=UnlessCondition(mock_hardware),
        target_container=name + "_container",
        composable_node_descriptions=[
            ComposableNode(
                package="image_proc",
                plugin="image_proc::RectifyNode",
                name=name + side + "_rectify_color_node",
                remappings=[
                    ("image", name + type + "/image_raw"),
                    ("camera_info", name + type + "/camera_info"),
                    ("image_rect", name + type + "/image_rect"),
                    ("image_rect/compressed", name + type + "/image_rect/compressed"),
                    (
                        "image_rect/compressedDepth",
                        name + type + "/image_rect/compressedDepth",
                    ),
                    ("image_rect/theora", name + type + "/image_rect/theora"),
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            )
        ],
    )


def create_spatial_bb_node(
    mock_hardware: LaunchConfiguration,
    name: str,
    cam_params_path: LaunchConfiguration,
) -> LoadComposableNodes:
    return LoadComposableNodes(
        condition=UnlessCondition(mock_hardware),
        target_container=name + "_container",
        composable_node_descriptions=[
            ComposableNode(
                package="depthai_filters",
                plugin="depthai_filters::SpatialBB",
                name=name + "_spatial_bb_node",
                remappings=[
                    ("stereo/camera_info", name + "/stereo/camera_info"),
                    ("nn/spatial_detections", name + "/nn/spatial_detections"),
                    ("rgb/preview/image_raw", name + "/rgb/preview/image_raw"),
                    ("overlay", name + "/overlay"),
                    ("spatial_bb", name + "/spatial_bb"),
                ],
                parameters=[cam_params_path],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
    )


def create_point_cloud_node(
    mock_hardware: LaunchConfiguration, name: str
) -> LoadComposableNodes:
    return LoadComposableNodes(
        condition=UnlessCondition(mock_hardware),
        target_container=name + "_container",
        composable_node_descriptions=[
            ComposableNode(
                package="depth_image_proc",
                plugin="depth_image_proc::PointCloudXyzrgbNode",
                name=name + "_point_cloud_xyzrgb_node",
                remappings=[
                    ("rgb/camera_info", name + "/rgb/camera_info"),
                    ("rgb/image_rect_color", name + "/rgb/image_rect"),
                    ("depth_registered/image_rect", name + "/stereo/image_raw"),
                    ("/points", name + "/pointcloud"),
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
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
