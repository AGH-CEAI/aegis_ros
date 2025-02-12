from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


class DepthAIConfig:
    def __init__(self):
        self.params_file = (
            LaunchConfiguration("params_file", default="oak_d_pro_scene"),
        )
        self.name_pro_scene = PathJoinSubstitution(
            [
                FindPackageShare("aegis_control"),
                "config",
                "cameras",
                "depthai_cameras.yaml",
            ]
        )
        # TODO(issue#26) create proper mock for the luxonis cameras
        self.mock_hardware = LaunchConfiguration("mock_hardware", default="true")

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


def generate_launch_description():
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
            "i_tf_tf_prefix": cfg.name_pro_scene,
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
    ]


def create_camera_node(
    mock_hardware: LaunchConfiguration,
    name: LaunchConfiguration,
    tf_params: dict,
    params_file: LaunchConfiguration,
    log_level: str,
) -> LoadComposableNodes:
    return ComposableNodeContainer(
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
    mock_hardware: LaunchConfiguration, name: LaunchConfiguration
) -> LoadComposableNodes:
    return LoadComposableNodes(
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
    name: LaunchConfiguration,
    params_file: LaunchConfiguration,
) -> LoadComposableNodes:
    return LoadComposableNodes(
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
