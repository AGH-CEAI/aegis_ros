from typing import Dict, List

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
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


class AegisPathsCfg:
    """Contains paths to the configuration files."""

    def __init__(self):
        self.moveit_cfg_pkg_name = "aegis_moveit_config"
        self.moveit_cfg_pkg = FindPackageShare(self.moveit_cfg_pkg_name)

        self.controllers_cfg = "config/move_group/controllers.yaml"
        self.joint_limits_cfg = "config/move_group/joint_limits.yaml"
        self.ompl_planning_cfg = "config/move_group/ompl_planning.yaml"

        self.kinematics_cfg = PathJoinSubstitution(
            [self.moveit_cfg_pkg, "config", "move_group", "kinematics.yaml"]
        )

        self.srdf_path = PathJoinSubstitution(
            [self.moveit_cfg_pkg, "urdf", "aegis.srdf"]
        )
        self.urdf_path = PathJoinSubstitution(
            [self.moveit_cfg_pkg, "urdf", "aegis.urdf.xacro"]
        )

        self.ur_calibrarion_cfg = PathJoinSubstitution(
            [self.moveit_cfg_pkg, "config", "ur_calibration.yaml"]
        )

        self.rviz_cfg = PathJoinSubstitution(
            [self.moveit_cfg_pkg, "config", "moveit.rviz"]
        )
        self.xacro_path = PathJoinSubstitution([FindExecutable(name="xacro")])


def generate_launch_description() -> LaunchDescription:

    declared_arguments = [
        DeclareLaunchArgument(
            "use_sim",
            default_value="false",
            description="Indicate whether robot is running in simulation or not.",
        ),
        DeclareLaunchArgument(
            "launch_rviz", default_value="true", description="Launch RViz?"
        ),
        # TODO(issue#5) enable real-time servo
        # DeclareLaunchArgument(
        #     "launch_servo", default_value="false", description="Launch Servo?"
        # ),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )


def launch_setup(context: LaunchContext) -> List[Node]:

    launch_rviz = LaunchConfiguration("launch_rviz")
    use_sim = LaunchConfiguration("use_sim")
    # TODO(issue#5) enable real-time servo
    # launch_servo = LaunchConfiguration("launch_servo")

    aegis_paths = AegisPathsCfg()
    robot_description = get_robot_description(aegis_paths)

    move_group_node, rviz_node = prepare_move_group_and_rviz_nodes(
        use_sim=use_sim,
        launch_rviz=launch_rviz,
        paths=aegis_paths,
        robot_description=robot_description,
    )

    tf_robot_base_node, tf_odom_node = prepare_static_transforms_nodes()
    robot_state_publisher_node = prepare_robot_state_publisher_node(robot_description)

    nodes_to_start = [
        move_group_node,
        rviz_node,
        tf_robot_base_node,
        tf_odom_node,
        robot_state_publisher_node,
        # TODO(issue#5) enable real-time servo
        # servo_node(),
        # TODO(issue#6) enable RGBD it for real hardware
        # rgbd_point_cloud_node(),
    ]

    return nodes_to_start


def get_robot_description(paths: AegisPathsCfg) -> Dict:
    robot_description_content = Command(
        [
            paths.xacro_path,
            " ",
            paths.urdf_path,
            " ",
            "kinematics_params:=",
            paths.ur_calibrarion_cfg,
        ]
    )
    return {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }


def get_robot_description_semantic(paths: AegisPathsCfg) -> Dict:
    robot_description_semantic_content = Command(
        [paths.xacro_path, " ", paths.srdf_path]
    )
    return {
        "robot_description_semantic": ParameterValue(
            robot_description_semantic_content, value_type=str
        )
    }


def prepare_move_group_and_rviz_nodes(
    use_sim: LaunchConfiguration,
    launch_rviz: LaunchConfiguration,
    paths: AegisPathsCfg,
    robot_description: Dict,
) -> tuple[Node, Node]:

    # TODO(issue#2) re-enable simulation
    # use_fake_hardware = use_sim

    robot_description_planning = {
        "robot_description_planning": load_yaml(
            paths.moveit_cfg_pkg_name,
            paths.joint_limits_cfg,
        )
    }

    # Planning Configuration
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }

    ompl_planning_yaml = load_yaml(paths.moveit_cfg_pkg_name, paths.ompl_planning_cfg)
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # Trajectory Execution Configuration
    controllers_yaml = load_yaml(paths.moveit_cfg_pkg_name, paths.controllers_cfg)
    # TODO(issue#2) use fake hardware for the simulation
    # the scaled_joint_trajectory_controller does not work on fake hardware
    # change_controllers = context.perform_substitution(use_fake_hardware)
    # if change_controllers == "true":
    #     controllers_yaml["scaled_joint_trajectory_controller"]["default"] = False
    #     controllers_yaml["joint_trajectory_controller"]["default"] = True

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
        "moveit_config_pkg": paths.moveit_cfg_pkg,
        "moveit_controllers": moveit_controllers,
        "ompl_planning_pipeline_config": ompl_planning_pipeline_config,
        "planning_scene_monitor_parameters": planning_scene_monitor_parameters,
        "robot_description_kinematics_file": paths.kinematics_cfg,
        "robot_description_planning": robot_description_planning,
        "robot_description_semantic": get_robot_description_semantic(paths),
        "robot_description": robot_description,
        "trajectory_execution": trajectory_execution,
        "use_sim": use_sim,
        "warehouse_ros_config": warehouse_ros_config,
    }

    return prepare_move_group_node(node_cfg), prepare_rviz_node(node_cfg)


def prepare_move_group_node(cfg: Dict) -> Node:
    return Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            cfg["robot_description"],
            cfg["robot_description_semantic"],
            cfg["robot_description_kinematics_file"],
            cfg["robot_description_planning"],
            cfg["ompl_planning_pipeline_config"],
            cfg["trajectory_execution"],
            cfg["moveit_controllers"],
            cfg["planning_scene_monitor_parameters"],
            {"use_sim_time": cfg["use_sim"]},
            {"publish_robot_description": True},
            {"publish_robot_description_semantic": True},
            # TODO(issue#1) Re-enable warehouse integration
            # cfg["warehouse_ros_config"],
        ],
    )


def prepare_rviz_node(cfg: Dict) -> Node:
    rviz_config_file = PathJoinSubstitution(
        [cfg["moveit_config_pkg"], "config", "moveit.rviz"]
    )
    return Node(
        package="rviz2",
        condition=IfCondition(cfg["launch_rviz"]),
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            cfg["robot_description"],
            cfg["robot_description_semantic"],
            cfg["ompl_planning_pipeline_config"],
            cfg["robot_description_kinematics_file"],
            cfg["robot_description_planning"],
            # TODO(issue#1) Re-enable warehouse integration
            # cfg["warehouse_ros_config"],
        ],
    )


def prepare_static_transforms_nodes() -> tuple[Node, Node]:
    tf_robot_base_node = static_tf_node("world", "ur_base")
    tf_odom_node = static_tf_node("world", "odom")
    return tf_robot_base_node, tf_odom_node


def static_tf_node(base_link: str, child_link: str) -> Node:
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", base_link, child_link],
    )


def prepare_robot_state_publisher_node(robot_description: Dict) -> Node:
    return Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
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
