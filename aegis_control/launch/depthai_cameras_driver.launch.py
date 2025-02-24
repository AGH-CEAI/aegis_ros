import os
import json
import yaml
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


class DepthAIConfig:
    def __init__(self):
        self.name_pro_scene = LaunchConfiguration(
            "name_pro_scene", default="oak_d_pro_scene"
        )
        self.params_file = PathJoinSubstitution(
            [
                FindPackageShare("aegis_control"),
                "config",
                "cameras",
                "depthai_cameras.yaml",
            ]
        )
        # TODO(issue#26) Introduce a mock for the DepthAI cameras
        self.mock_hardware = LaunchConfiguration("mock_hardware", default="false")

        self.cam_model_pro_scene = LaunchConfiguration(
            "camera_model_pro_scene", default="OAK-D-S2"
        )
        self.parent_frame_pro_scene = LaunchConfiguration(
            "parent_frame_pro_scene", default="cell"
        )
        self.base_frame_pro_scene = LaunchConfiguration(
            "base_frame_pro_scene", default="oak_d_pro_scene_frame"
        )
        self.cam_pos_x_pro_scene = LaunchConfiguration(
            "cam_pos_x_pro_scene", default="0.014"
        )
        self.cam_pos_y_pro_scene = LaunchConfiguration(
            "cam_pos_y_pro_scene", default="0.33"
        )
        self.cam_pos_z_pro_scene = LaunchConfiguration(
            "cam_pos_z_pro_scene", default="1.972"
        )
        self.cam_roll_pro_scene = LaunchConfiguration(
            "cam_roll_pro_scene", default="1.5708"
        )
        self.cam_pitch_pro_scene = LaunchConfiguration(
            "cam_pitch_pro_scene", default="1.5708"
        )
        self.cam_yaw_pro_scene = LaunchConfiguration("cam_yaw_pro_scene", default="0")


def generate_launch_description() -> LaunchDescription:
    modify_config()
    return LaunchDescription([OpaqueFunction(function=launch_setup)])


def launch_setup(context) -> list[Node]:
    # TODO(issue#22): Setup global log level configuration
    log_level = "info"
    if context.environment.get("DEPTHAI_DEBUG") == "1":
        log_level = "debug"

    cfg = DepthAIConfig()
    name_pro_scene_str = cfg.name_pro_scene.perform(context)

    # TODO(issue#23): Investigate the necessity of tf parameters
    tf_params_pro_scene = {
        "camera": {
            "i_publish_tf_from_calibration": False,
            "i_tf_tf_prefix": name_pro_scene_str,
            "i_tf_camera_model": cfg.cam_model_pro_scene,
            "i_tf_parent_frame": cfg.parent_frame_pro_scene.perform(context),
            "i_tf_base_frame": cfg.base_frame_pro_scene.perform(context),
            "i_tf_cam_pos_x": cfg.cam_pos_x_pro_scene.perform(context),
            "i_tf_cam_pos_y": cfg.cam_pos_y_pro_scene.perform(context),
            "i_tf_cam_pos_z": cfg.cam_pos_z_pro_scene.perform(context),
            "i_tf_cam_roll": cfg.cam_roll_pro_scene.perform(context),
            "i_tf_cam_pitch": cfg.cam_pitch_pro_scene.perform(context),
            "i_tf_cam_yaw": cfg.cam_yaw_pro_scene.perform(context),
        }
    }

    return [
        create_camera_node(
            cfg.mock_hardware,
            name_pro_scene_str,
            tf_params_pro_scene,
            cfg.params_file,
            log_level,
        ),
        create_rectify_node(cfg.mock_hardware, name_pro_scene_str),
        create_spatial_bb_node(cfg.mock_hardware, name_pro_scene_str, cfg.params_file),
        create_point_cloud_node(cfg.mock_hardware, name_pro_scene_str),
    ]


def create_camera_node(
    mock_hardware: LaunchConfiguration,
    name: str,
    tf_params: dict,
    params_file: LaunchConfiguration,
    log_level: str,
) -> LoadComposableNodes:
    return ComposableNodeContainer(
        condition=UnlessCondition(mock_hardware),
        name=name + "_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="depthai_ros_driver",
                plugin="depthai_ros_driver::Camera",
                name=name,
                parameters=[params_file, tf_params],
            )
        ],
        arguments=["--ros-args", "--log-level", log_level],
        output="both",
    )


def create_rectify_node(
    mock_hardware: LaunchConfiguration, name: str
) -> LoadComposableNodes:
    return LoadComposableNodes(
        condition=UnlessCondition(mock_hardware),
        target_container=name + "_container",
        composable_node_descriptions=[
            ComposableNode(
                package="image_proc",
                plugin="image_proc::RectifyNode",
                name=name + "_rectify_color_node",
                remappings=[
                    ("image", name + "/rgb/image_raw"),
                    ("camera_info", name + "/rgb/camera_info"),
                    ("image_rect", name + "/rgb/image_rect"),
                    ("image_rect/compressed", name + "/rgb/image_rect/compressed"),
                    (
                        "image_rect/compressedDepth",
                        name + "/rgb/image_rect/compressedDepth",
                    ),
                    ("image_rect/theora", name + "/rgb/image_rect/theora"),
                ],
            )
        ],
    )


def create_spatial_bb_node(
    mock_hardware: LaunchConfiguration,
    name: str,
    params_file: LaunchConfiguration,
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
                parameters=[params_file],
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
            ),
        ],
    )


def modify_config() -> None:
    # TODO(issue#27) Fix YOLO config not being applied correctly
    package_share_path = FindPackageShare("aegis_control").find("aegis_control")

    model_path = os.path.join(os.path.expanduser("~"), "ceai_models", "yolo.blob")
    yolo_cfg_path = os.path.join(package_share_path, "config", "cameras", "yolo.json")
    cam_cfg_path = os.path.join(
        package_share_path, "config", "cameras", "depthai_cameras.yaml"
    )

    with open(yolo_cfg_path, "r") as file:
        yolo_cfg = json.load(file)

    yolo_cfg["model"]["model_name"] = model_path

    with open(yolo_cfg_path, "w") as file:
        json.dump(yolo_cfg, file, indent=2)

    with open(cam_cfg_path, "r") as file:
        cam_cfg = yaml.safe_load(file)

    cam_cfg["/oak_d_pro_scene"]["ros__parameters"]["nn"]["i_nn_config_path"] = (
        yolo_cfg_path
    )

    with open(cam_cfg_path, "w") as file:
        yaml.safe_dump(cam_cfg, file)
