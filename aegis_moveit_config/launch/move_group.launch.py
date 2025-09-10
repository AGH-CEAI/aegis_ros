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

# from launch.event_handlers import OnProcessExit
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def str2bool(x: str) -> bool:
    return x.lower() in ("true")


def deep_dict_update(d_target: dict, d_update: dict) -> dict:
    for k, v in d_update.items():
        if k in d_target and isinstance(d_target[k], dict) and isinstance(v, dict):
            deep_dict_update(d_target[k], v)
        else:
            d_target[k] = v
    return d_target


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

    def load_planning_joint_limits_cfg(self) -> dict:
        return load_yaml(
            self.moveit_cfg_pkg_name, "config/move_group/planning_joint_limits.yaml"
        )

    def load_octomap_updater_cfg(self) -> dict:
        return load_yaml(self.moveit_cfg_pkg_name, "config/octomap_updater.yaml")


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([OpaqueFunction(function=launch_setup)])


def launch_setup(context: LaunchContext) -> list[Node]:
    mock_hardware = LaunchConfiguration("mock_hardware")
    launch_rviz = LaunchConfiguration("launch_rviz")
    # TODO(issue#5) enable real-time servo
    launch_servo = LaunchConfiguration("launch_servo")

    paths = AegisPathsCfg()

    mock_hardware_bool = str2bool(context.perform_substitution(mock_hardware))

    robot_description = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("aegis_description"),
                    "urdf",
                    "aegis.urdf.xacro",
                ]
            ),
            " ",
            "tf_prefix:=",
            " ",
            " ",
            "mock_hardware:=",
            mock_hardware,
        ]
    )

    # Planning joints limits
    robot_description_planning = {
        "robot_description_planning": deep_dict_update(
            paths.load_joint_limits_cfg(),
            paths.load_planning_joint_limits_cfg(),
        )
    }

    # Planning Configuration
    ompl_planning_pipeline_cfg = paths.load_ompl_planning_cfg()

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
        # UR Driver is not compatible with the MoveIt's Trajectory Execution Monitoring (TEM)
        # See # https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_robot_driver/ur_moveit_config/doc/index.html#id2
        # Execution time monitoring can be incompatible with the scaled JTC
        "trajectory_execution.execution_duration_monitoring": False,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    octomap_parameters = {
        "frame_id": "world",
        "resolution": 0.01,
        "max_range": 2.0,
    }

    octomap_updater_parameters = paths.load_octomap_updater_cfg()

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
        "ompl_planning_pipeline_config": ompl_planning_pipeline_cfg,
        "planning_scene_monitor_parameters": planning_scene_monitor_parameters,
        "robot_description": robot_description,
        "robot_description_kinematics_file": paths.kinematics_cfg,
        "robot_description_planning": robot_description_planning,
        "robot_description_semantic": get_robot_description_semantic(paths),
        "trajectory_execution": trajectory_execution,
        "octomap_parameters": octomap_parameters,
        "octomap_updater_parameters": octomap_updater_parameters,
        "mock_hardware": mock_hardware,
        "warehouse_ros_config": warehouse_ros_config,
    }

    move_group_node = prepare_move_group_node(node_cfg)
    rviz_node = prepare_rviz_node(node_cfg, paths)
    tf_odom_node = prepare_static_tf_node("world", "odom")
    scene_objects_manager_node = prepare_scene_objects_manager_node(paths)
    octomap_node = prepare_octomap_node(node_cfg)

    # Launch as much as possible in components
    container = ComposableNodeContainer(
        name="moveit_servo_demo_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="aegis_moveit_config",
                plugin="aegis_moveit_config::JoyToServoPubAegis",
                name="controller_to_servo_node",
            ),
            ComposableNode(
                package="joy",
                plugin="joy::Joy",
                name="joy_node",
            ),
        ],
        output="screen",
    )

    return [
        move_group_node,
        rviz_node,
        tf_odom_node,
        scene_objects_manager_node,
        octomap_node,
        # TODO(issue#5) enable real-time servo
        servo_node(node_cfg),
        container,
    ]


def get_robot_description_semantic(paths: AegisPathsCfg) -> dict:
    robot_description_semantic_content = Command(
        [paths.xacro_path, " ", paths.srdf_path]
    )
    return {
        "robot_description_semantic": ParameterValue(
            robot_description_semantic_content, value_type=str
        )
    }


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
            cfg["octomap_parameters"],
            cfg["octomap_updater_parameters"],
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


def prepare_octomap_node(cfg: dict) -> Node:
    return Node(
        package="octomap_server",
        executable="octomap_server_node",
        output="screen",
        parameters=[cfg["octomap_parameters"]],
        remappings=[("/cloud_in", "/cam_scene/pointcloud")],
    )


# TODO(issue#1) Re-enable warehouse integration
# def warehouse_cfg() -> Dict[str, str]:
#     warehouse_sqlite_path = LaunchConfiguration("warehouse_sqlite_path")
#     return {
#         "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
#         "warehouse_host": warehouse_sqlite_path,
#     }


# TODO(issue#5) Enable MoveIt servo
def servo_node(cfg: dict) -> Node:
    print(f"Type{type(cfg['robot_description'])}")
    # Servo node for realtime control
    servo_yaml = load_yaml("aegis_moveit_config", "config/move_group/ur_servo.yaml")
    print(f"Servo: {servo_yaml}")
    servo_params = {"moveit_servo": servo_yaml}
    return Node(
        package="moveit_servo",
        # condition=IfCondition(launch_servo),
        executable="servo_node_main",
        parameters=[
            servo_params,
            {
                "robot_description": ParameterValue(
                    cfg["robot_description"], value_type=str
                )
            },
            cfg["robot_description_semantic"],
            cfg["robot_description_kinematics_file"],
        ],
        output="screen",
    )
