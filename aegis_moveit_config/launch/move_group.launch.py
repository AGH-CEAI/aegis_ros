from launch import LaunchDescription, LaunchContext
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.descriptions import ParameterValue
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ur_moveit_config.launch_common import load_yaml


def str2bool(x: str) -> bool:
    return x.lower() in ("true")


class AegisPathsCfg:
    """Contains paths to the configuration files."""

    def __init__(self):
        self.moveit_cfg_pkg_name = "aegis_moveit_config"
        self.moveit_cfg_pkg = FindPackageShare(self.moveit_cfg_pkg_name)

        self.description_cfg_pkg_name = "aegis_description"
        self.description_cfg_pkg = FindPackageShare(self.description_cfg_pkg_name)

        self.kinematics_cfg = PathJoinSubstitution(
            [self.moveit_cfg_pkg, "config", "move_group", "kinematics.yaml"]
        )

        self.srdf_path = PathJoinSubstitution(
            [self.moveit_cfg_pkg, "config", "aegis.srdf"]
        )

        self.ur_calibrarion_cfg = PathJoinSubstitution(
            [self.description_cfg_pkg, "config", "ur5e", "calibration.yaml"]
        )

        self.scene_objects_cfg = PathJoinSubstitution(
            [self.moveit_cfg_pkg, "config", "scene_objects.yaml"]
        )

        self.rviz_cfg = PathJoinSubstitution(
            [self.moveit_cfg_pkg, "config", "moveit.rviz"]
        )
        self.xacro_path = PathJoinSubstitution([FindExecutable(name="xacro")])

    def load_ompl_planning_cfg(self) -> dict:
        return load_yaml(
            self.moveit_cfg_pkg_name, "config/move_group/ompl_planning.yaml"
        )

    def load_controllers_cfg(self) -> dict:
        return load_yaml(self.moveit_cfg_pkg_name, "config/controlers_description.yaml")

    def load_joint_limits_cfg(self) -> dict:
        return load_yaml(self.description_cfg_pkg_name, "config/ur5e/joint_limits.yaml")


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([OpaqueFunction(function=launch_setup)])


def launch_setup(context: LaunchContext) -> list[Node]:
    mock_hardware = LaunchConfiguration("mock_hardware")
    launch_rviz = LaunchConfiguration("launch_rviz")
    # TODO(issue#5) enable real-time servo
    # launch_servo = LaunchConfiguration("launch_servo")

    aegis_paths = AegisPathsCfg()

    move_group_node, rviz_node = prepare_move_group_and_rviz_nodes(
        mock_hardware=mock_hardware,
        mock_hardware_bool=str2bool(context.perform_substitution(mock_hardware)),
        launch_rviz=launch_rviz,
        paths=aegis_paths,
    )

    tf_odom_node = prepare_static_tf_node("world", "odom")
    scene_objects_manager_node = prepare_scene_objects_manager_node(aegis_paths)

    nodes_to_start = [
        move_group_node,
        rviz_node,
        tf_odom_node,
        scene_objects_manager_node,
        # TODO(issue#5) enable real-time servo
        # servo_node(),
        # TODO(issue#6) enable RGBD it for real hardware
        # rgbd_point_cloud_node(),
    ]

    return nodes_to_start


def get_robot_description_semantic(paths: AegisPathsCfg) -> dict:
    robot_description_semantic_content = Command(
        [paths.xacro_path, " ", paths.srdf_path]
    )
    return {
        "robot_description_semantic": ParameterValue(
            robot_description_semantic_content, value_type=str
        )
    }


def prepare_move_group_and_rviz_nodes(
    mock_hardware: LaunchConfiguration,
    mock_hardware_bool: bool,
    launch_rviz: LaunchConfiguration,
    paths: AegisPathsCfg,
) -> tuple[Node, Node]:
    robot_description_planning = {
        "robot_description_planning": paths.load_joint_limits_cfg()
    }

    # Planning Configuration
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }

    ompl_planning_yaml = paths.load_ompl_planning_cfg()
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # Trajectory Execution Configuration
    controllers_yaml = paths.load_controllers_cfg()

    # TODO(issue#11) Extract configuration for real/fake controller to an external YAML
    if mock_hardware_bool:
        # the scaled_joint_trajectory_controller does not work on fake hardware
        controllers_yaml["scaled_joint_trajectory_controller"]["default"] = False
        controllers_yaml["joint_trajectory_controller"]["default"] = True

    moveit_controllers = {
        "moveit_simple_controller_manager": controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # TODO(issue#1) integrate the warehouse with the Aegis setup
    # warehouse_ros_config = {
    #     "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
    #     "warehouse_host": "/home/macale/ceai/db/warehouse_db.sqlite",
    #     "port": 33828,
    #     "scene_name": "",  # If scene name is empty, all scenes will be used
    #     "queries_regex": ".*",
    # }
    warehouse_ros_config = None

    node_cfg = {
        "launch_rviz": launch_rviz,
        "moveit_controllers": moveit_controllers,
        "ompl_planning_pipeline_config": ompl_planning_pipeline_config,
        "planning_scene_monitor_parameters": planning_scene_monitor_parameters,
        "robot_description_kinematics_file": paths.kinematics_cfg,
        "robot_description_planning": robot_description_planning,
        "robot_description_semantic": get_robot_description_semantic(paths),
        "trajectory_execution": trajectory_execution,
        "mock_hardware": mock_hardware,
        "warehouse_ros_config": warehouse_ros_config,
    }

    return prepare_move_group_node(node_cfg), prepare_rviz_node(node_cfg, paths)


def prepare_move_group_node(cfg: dict) -> Node:
    return Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            cfg["robot_description_semantic"],
            cfg["robot_description_kinematics_file"],
            cfg["robot_description_planning"],
            cfg["ompl_planning_pipeline_config"],
            cfg["trajectory_execution"],
            cfg["moveit_controllers"],
            cfg["planning_scene_monitor_parameters"],
            {"use_sim_time": cfg["mock_hardware"]},
            {"publish_robot_description": True},
            {"publish_robot_description_semantic": True},
            # TODO(issue#1) Re-enable warehouse integration
            # cfg["warehouse_ros_config"],
        ],
    )


def prepare_rviz_node(cfg: dict, paths: AegisPathsCfg) -> Node:
    return Node(
        package="rviz2",
        condition=IfCondition(cfg["launch_rviz"]),
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", paths.rviz_cfg],
        parameters=[
            cfg["robot_description_semantic"],
            cfg["ompl_planning_pipeline_config"],
            cfg["robot_description_kinematics_file"],
            cfg["robot_description_planning"],
            # TODO(issue#1) Re-enable warehouse integration
            # cfg["warehouse_ros_config"],
        ],
    )


def prepare_static_tf_node(base_link: str, child_link: str) -> Node:
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=[
            "--x",
            "0.0",
            "--y",
            "0.0",
            "--z",
            "0.0",
            "--roll",
            "0.0",
            "--pitch",
            "0.0",
            "--yaw",
            "0.0",
            "--frame-id",
            base_link,
            "--child-frame-id",
            child_link,
        ],
    )


def prepare_scene_objects_manager_node(paths: AegisPathsCfg) -> Node:
    return Node(
        package="scene_objects_manager",
        executable="scene_objects_manager",
        name="scene_objects_manager",
        output="screen",
        arguments=["--cfg", paths.scene_objects_cfg, "--frame", "ur_base"],
    )


# TODO(issue#1) Re-enable warehouse integration
# def warehouse_cfg() -> Dict[str, str]:
#     warehouse_sqlite_path = LaunchConfiguration("warehouse_sqlite_path")
#     return {
#         "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
#         "warehouse_host": warehouse_sqlite_path,
#     }

# TODO(issue#5) Enable MoveIt servo
# def servo_node() -> Node:
#     # Servo node for realtime control
#     servo_yaml = load_yaml("aegis_moveit_config", "config/ur_servo.yaml")
#     servo_params = {"moveit_servo": servo_yaml}
#     return Node(
#         package="moveit_servo",
#         condition=IfCondition(launch_servo),
#         executable="servo_node_main",
#         parameters=[
#             servo_params,
#             robot_description,
#             robot_description_semantic,
#         ],
#         output="screen",
#     )

# TODO(issue#6) enable RGBD it for real hardware
# def rgbd_point_cloud_node() -> Node:
#     return ComposableNodeContainer(
#         name="container0",
#         namespace="",
#         package="rclcpp_components",
#         executable="component_container",
#         composable_node_descriptions=[
#             ComposableNode(
#                 package="depth_image_proc",
#                 plugin="depth_image_proc::PointCloudXyzrgbNode",
#                 name="point_cloud_xyzrgb_node",
#                 remappings=[
#                     ("rgb/camera_info", "/color_camera_info"),
#                     ("rgb/image_rect_color", "/camera_image_color"),
#                     ("depth_registered/image_rect", "/camera_image_depth"),
#                     ("/points", "/pointcloud"),
#                 ],
#             ),
#         ],
#         output="screen",
#         parameters=[{"use_sim_time": True}],
#     )
