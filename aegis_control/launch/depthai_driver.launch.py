from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def create_camera_node(name, tf_params, params_file, log_level):
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


def create_rectify_node(name):
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


def create_spatial_bb_node(name, params_file):
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


def launch_setup(context, *args, **kwargs):
    log_level = "info"
    if context.environment.get("DEPTHAI_DEBUG") == "1":
        log_level = "debug"

    params_file = LaunchConfiguration("params_file")

    name_pro = LaunchConfiguration("name_pro").perform(context)
    cam_model_pro = LaunchConfiguration("camera_model_pro", default="OAK-D-S2")
    parent_frame_pro = LaunchConfiguration("parent_frame_pro", default="cell")
    base_frame_pro = LaunchConfiguration("base_frame_pro", default="oak_d_pro_frame")
    cam_pos_x_pro = LaunchConfiguration("cam_pos_x_pro", default="0.014")
    cam_pos_y_pro = LaunchConfiguration("cam_pos_y_pro", default="0.33")
    cam_pos_z_pro = LaunchConfiguration("cam_pos_z_pro", default="1.972")
    cam_roll_pro = LaunchConfiguration("cam_roll_pro", default="1.5708")
    cam_pitch_pro = LaunchConfiguration("cam_pitch_pro", default="1.5708")
    cam_yaw_pro = LaunchConfiguration("cam_yaw_pro", default="0.0")

    tf_params_pro = {
        "camera": {
            "i_publish_tf_from_calibration": True,
            "i_tf_tf_prefix": name_pro,
            "i_tf_camera_model": cam_model_pro,
            "i_tf_parent_frame": parent_frame_pro.perform(context),
            "i_tf_base_frame": base_frame_pro.perform(context),
            "i_tf_cam_pos_x": cam_pos_x_pro.perform(context),
            "i_tf_cam_pos_y": cam_pos_y_pro.perform(context),
            "i_tf_cam_pos_z": cam_pos_z_pro.perform(context),
            "i_tf_cam_roll": cam_roll_pro.perform(context),
            "i_tf_cam_pitch": cam_pitch_pro.perform(context),
            "i_tf_cam_yaw": cam_yaw_pro.perform(context),
        }
    }

    return [
        create_camera_node(name_pro, tf_params_pro, params_file, log_level),
        create_rectify_node(name_pro),
        create_spatial_bb_node(name_pro, params_file),
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("name_pro", default_value="oak_d_pro"),
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("aegis_control"),
                    "config",
                    "camera.yaml",
                ]
            ),
        ),
    ]
    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
