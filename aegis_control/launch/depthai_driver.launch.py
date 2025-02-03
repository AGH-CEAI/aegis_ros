from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("name_pro_scene", default_value="oak_d_pro_scene"),
        DeclareLaunchArgument("name_sr_left", default_value="oak_d_sr_left"),
        DeclareLaunchArgument("name_sr_right", default_value="oak_d_sr_right"),
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


def launch_setup(context):
    log_level = "info"
    if context.environment.get("DEPTHAI_DEBUG") == "1":
        log_level = "debug"

    params_file = LaunchConfiguration("params_file")

    name_pro_scene = LaunchConfiguration("name_pro_scene").perform(context)
    cam_model_pro_scene = LaunchConfiguration(
        "camera_model_pro_scene", default="OAK-D-S2"
    )
    parent_frame_pro_scene = LaunchConfiguration(
        "parent_frame_pro_scene", default="cell"
    )
    base_frame_pro_scene = LaunchConfiguration(
        "base_frame_pro_scene", default="oak_d_pro_scene_frame"
    )
    cam_pos_x_pro_scene = LaunchConfiguration("cam_pos_x_pro_scene", default="0.014")
    cam_pos_y_pro_scene = LaunchConfiguration("cam_pos_y_pro_scene", default="0.33")
    cam_pos_z_pro_scene = LaunchConfiguration("cam_pos_z_pro_scene", default="1.972")
    cam_roll_pro_scene = LaunchConfiguration("cam_roll_pro_scene", default="1.5708")
    cam_pitch_pro_scene = LaunchConfiguration("cam_pitch_pro_scene", default="1.5708")
    cam_yaw_pro_scene = LaunchConfiguration("cam_yaw_pro_scene", default="0.0")

    name_sr_left = LaunchConfiguration("name_sr_left").perform(context)
    cam_model_sr_left = LaunchConfiguration(
        "camera_model_sr_left", default="OAK-D-SR-POE"
    )
    parent_frame_sr_left = LaunchConfiguration(
        "parent_frame_sr_left", default="adapter_to_sensor"
    )
    base_frame_sr_left = LaunchConfiguration(
        "base_frame_sr_left", default="oak_d_sr_left_frame"
    )
    cam_pos_x_sr_left = LaunchConfiguration("cam_pos_x_sr_left", default="0.1")
    cam_pos_y_sr_left = LaunchConfiguration("cam_pos_y_sr_left", default="0")
    cam_pos_z_sr_left = LaunchConfiguration("cam_pos_z_sr_left", default="0.14")
    cam_roll_sr_left = LaunchConfiguration("cam_roll_sr_left", default="0")
    cam_pitch_sr_left = LaunchConfiguration("cam_pitch_sr_left", default="-1.8326")
    cam_yaw_sr_left = LaunchConfiguration("cam_yaw_sr_left", default="0")

    name_sr_right = LaunchConfiguration("name_sr_right").perform(context)
    cam_model_sr_right = LaunchConfiguration(
        "camera_model_sr_right", default="OAK-D-SR-POE"
    )
    parent_frame_sr_right = LaunchConfiguration(
        "parent_frame_sr_right", default="adapter_to_sensor"
    )
    base_frame_sr_right = LaunchConfiguration(
        "base_frame_sr_right", default="oak_d_sr_right_frame"
    )
    cam_pos_x_sr_right = LaunchConfiguration("cam_pos_x_sr_right", default="-0.1")
    cam_pos_y_sr_right = LaunchConfiguration("cam_pos_y_sr_right", default="0")
    cam_pos_z_sr_right = LaunchConfiguration("cam_pos_z_sr_right", default="0.14")
    cam_roll_sr_right = LaunchConfiguration("cam_roll_sr_right", default="3.1416")
    cam_pitch_sr_right = LaunchConfiguration("cam_pitch_sr_right", default="-1.3090")
    cam_yaw_sr_right = LaunchConfiguration("cam_yaw_sr_right", default="0")

    tf_params_pro_scene = {
        "camera": {
            "i_publish_tf_from_calibration": True,
            "i_tf_tf_prefix": name_pro_scene,
            "i_tf_camera_model": cam_model_pro_scene,
            "i_tf_parent_frame": parent_frame_pro_scene.perform(context),
            "i_tf_base_frame": base_frame_pro_scene.perform(context),
            "i_tf_cam_pos_x": cam_pos_x_pro_scene.perform(context),
            "i_tf_cam_pos_y": cam_pos_y_pro_scene.perform(context),
            "i_tf_cam_pos_z": cam_pos_z_pro_scene.perform(context),
            "i_tf_cam_roll": cam_roll_pro_scene.perform(context),
            "i_tf_cam_pitch": cam_pitch_pro_scene.perform(context),
            "i_tf_cam_yaw": cam_yaw_pro_scene.perform(context),
        }
    }

    tf_params_sr_left = {
        "camera": {
            "i_publish_tf_from_calibration": True,
            "i_tf_tf_prefix": name_sr_left,
            "i_tf_camera_model": cam_model_sr_left,
            "i_tf_parent_frame": parent_frame_sr_left.perform(context),
            "i_tf_base_frame": base_frame_sr_left.perform(context),
            "i_tf_cam_pos_x": cam_pos_x_sr_left.perform(context),
            "i_tf_cam_pos_y": cam_pos_y_sr_left.perform(context),
            "i_tf_cam_pos_z": cam_pos_z_sr_left.perform(context),
            "i_tf_cam_roll": cam_roll_sr_left.perform(context),
            "i_tf_cam_pitch": cam_pitch_sr_left.perform(context),
            "i_tf_cam_yaw": cam_yaw_sr_left.perform(context),
        }
    }

    tf_params_sr_right = {
        "camera": {
            "i_publish_tf_from_calibration": True,
            "i_tf_tf_prefix": name_sr_right,
            "i_tf_camera_model": cam_model_sr_right,
            "i_tf_parent_frame": parent_frame_sr_right.perform(context),
            "i_tf_base_frame": base_frame_sr_right.perform(context),
            "i_tf_cam_pos_x": cam_pos_x_sr_right.perform(context),
            "i_tf_cam_pos_y": cam_pos_y_sr_right.perform(context),
            "i_tf_cam_pos_z": cam_pos_z_sr_right.perform(context),
            "i_tf_cam_roll": cam_roll_sr_right.perform(context),
            "i_tf_cam_pitch": cam_pitch_sr_right.perform(context),
            "i_tf_cam_yaw": cam_yaw_sr_right.perform(context),
        }
    }

    return [
        create_camera_node(name_pro_scene, tf_params_pro_scene, params_file, log_level),
        create_camera_node(name_sr_left, tf_params_sr_left, params_file, log_level),
        create_camera_node(name_sr_right, tf_params_sr_right, params_file, log_level),
        create_rectify_node(name_pro_scene),
        create_spatial_bb_node(name_pro_scene, params_file),
    ]


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
