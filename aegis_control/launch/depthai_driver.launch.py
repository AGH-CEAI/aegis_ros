import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def launch_setup(context, *args, **kwargs):
    log_level = "info"
    if context.environment.get("DEPTHAI_DEBUG") == "1":
        log_level = "debug"

    params_file = LaunchConfiguration("params_file")
    name = LaunchConfiguration("name").perform(context)
    cam_model = LaunchConfiguration("camera_model", default="OAK-D-S2")
    parent_frame = LaunchConfiguration("parent_frame", default="world")
    base_frame = LaunchConfiguration("base_frame", default="oak-d_frame")
    cam_pos_x = LaunchConfiguration("cam_pos_x", default="0.0")
    cam_pos_y = LaunchConfiguration("cam_pos_y", default="0.0")
    cam_pos_z = LaunchConfiguration("cam_pos_z", default="0.0")
    cam_roll = LaunchConfiguration("cam_roll", default="0.0")
    cam_pitch = LaunchConfiguration("cam_pitch", default="0.0")
    cam_yaw = LaunchConfiguration("cam_yaw", default="0.0")
    publish_tf_from_calibration = LaunchConfiguration(
        "publish_tf_from_calibration", default="true"
    )
    rgb_topic_name = name + "/rgb/image_raw"
    rgb_topic_name = name + "/rgb/image_rect"

    tf_params = {}
    if publish_tf_from_calibration.perform(context) == "true":
        tf_params = {
            "camera": {
                "i_publish_tf_from_calibration": True,
                "i_tf_tf_prefix": name,
                "i_tf_camera_model": cam_model,
                "i_tf_parent_frame": parent_frame.perform(context),
                "i_tf_base_frame": base_frame.perform(context),
                "i_tf_cam_pos_x": cam_pos_x.perform(context),
                "i_tf_cam_pos_y": cam_pos_y.perform(context),
                "i_tf_cam_pos_z": cam_pos_z.perform(context),
                "i_tf_cam_roll": cam_roll.perform(context),
                "i_tf_cam_pitch": cam_pitch.perform(context),
                "i_tf_cam_yaw": cam_yaw.perform(context),
            }
        }

    return [
        ComposableNodeContainer(
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
        ),
        LoadComposableNodes(
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
        ),
        LoadComposableNodes(
            target_container=name + "_container",
            composable_node_descriptions=[
                ComposableNode(
                    package="depth_image_proc",
                    plugin="depth_image_proc::PointCloudXyzrgbNode",
                    name=name + "_point_cloud_xyzrgb_node",
                    remappings=[
                        ("depth_registered/image_rect", name + "/stereo/image_raw"),
                        ("rgb/image_rect_color", rgb_topic_name),
                        ("rgb/camera_info", name + "/rgb/camera_info"),
                        ("points", name + "/points"),
                    ],
                ),
            ],
        ),
        LoadComposableNodes(
            target_container=name + "_container",
            composable_node_descriptions=[
                ComposableNode(
                    package="depthai_filters",
                    plugin="depthai_filters::SpatialBB",
                    name="spatial_bb_node",
                    remappings=[
                        ("stereo/camera_info", name + "/stereo/camera_info"),
                        ("nn/spatial_detections", name + "/nn/spatial_detections"),
                        ("rgb/preview/image_raw", name + "/rgb/preview/image_raw"),
                    ],
                    parameters=[params_file],
                ),
            ],
        ),
    ]


def generate_launch_description():
    depthai_prefix = get_package_share_directory("aegis_control")
    declared_arguments = [
        DeclareLaunchArgument("name", default_value="oak"),
        DeclareLaunchArgument("camera_model", default_value="OAK-D-S2"),
        DeclareLaunchArgument("parent_frame", default_value="cell"),
        DeclareLaunchArgument("base_frame", default_value="oak-d_frame"),
        DeclareLaunchArgument("cam_pos_x", default_value="0.014"),
        DeclareLaunchArgument("cam_pos_y", default_value="0.33"),
        DeclareLaunchArgument("cam_pos_z", default_value="1.972"),
        DeclareLaunchArgument("cam_roll", default_value="1.5708"),
        DeclareLaunchArgument("cam_pitch", default_value="1.5708"),
        DeclareLaunchArgument("cam_yaw", default_value="0.0"),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(depthai_prefix, "config", "camera.yaml"),
        ),
        DeclareLaunchArgument(
            "publish_tf_from_calibration",
            default_value="true",
            description="Enables TF publishing from camera calibration file.",
        ),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
