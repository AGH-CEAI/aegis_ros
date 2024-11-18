from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    ld = LaunchDescription()

    pkg_path = FindPackageShare("aegis_description")
    model_path = PathJoinSubstitution(["urdf", "aegis.urdf.xacro"])
    rviz_config_path = PathJoinSubstitution([pkg_path, "rviz", "urdf.rviz"])
    run_joint_state_publisher_gui = "true"

    ld.add_action(
        IncludeLaunchDescription(
            PathJoinSubstitution(
                [FindPackageShare("urdf_launch"), "launch", "display.launch.py"]
            ),
            launch_arguments={
                "urdf_package": "aegis_description",
                "urdf_package_path": model_path,
                "rviz_config": rviz_config_path,
                "jsp_gui": run_joint_state_publisher_gui,
            }.items(),
        )
    )
    ld.add_action(static_tf_node("world", "ur_base"))

    return ld


def static_tf_node(base_link: str, child_link: str) -> Node:
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", base_link, child_link],
    )
