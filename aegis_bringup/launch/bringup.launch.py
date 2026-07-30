#!/usr/bin/env python3

from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def launch_setup(context: LaunchContext) -> list[IncludeLaunchDescription]:
    namespace = LaunchConfiguration("namespace", default="")
    launch_args = {
        "tf_prefix": LaunchConfiguration("tf_prefix", default=""),
        "mock_hardware": LaunchConfiguration("mock_hardware"),
        "launch_rviz": LaunchConfiguration("launch_rviz"),
        "disable_cameras": LaunchConfiguration("disable_cameras"),
    }

    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("aegis_description"),
                    "launch",
                    "robot_description.launch.py",
                ]
            )
        ),
        launch_arguments=launch_args.items(),
    )

    drivers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("aegis_control"), "launch", "start_drivers.launch.py"]
            )
        ),
        launch_arguments=launch_args.items(),
    )

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("aegis_moveit_config"),
                    "launch",
                    "move_group.launch.py",
                ]
            )
        ),
        launch_arguments=launch_args.items(),
    )

    grpc_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("aegis_grpc"),
                    "launch",
                    "start_server.launch.py",
                ]
            )
        ),
        launch_arguments=launch_args.items(),
    )

    # TODO(issue#41): enable aegis_director in the bringup's launch when the workflow will be ready
    director_launch = IncludeLaunchDescription(  # noqa: F841
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("aegis_director"),
                    "launch",
                    "director.launch.py",
                ]
            )
        ),
        launch_arguments=launch_args.items(),
    )

    return set_namespace(
        namespace,
        [
            robot_description_launch,
            drivers_launch,
            moveit_launch,
            grpc_server_launch,
            # TODO(issue#41): enable aegis_director in the bringup's launch when the workflow will be ready
            # director_launch,
        ],
    )


def set_namespace(namespace: LaunchConfiguration, description: list) -> list:
    return [
        GroupAction(actions=[PushRosNamespace(namespace), desc]) for desc in description
    ]


def generate_launch_description() -> LaunchDescription:
    declared_arguments = []

    # TODO(issue#13) Debug the namespace and tf_prefix launch args
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         "namespace",
    #         default_value=EnvironmentVariable("ROBOT_NAMESPACE", default_value=""),
    #         description="Add namespace to all launched nodes.",
    #     )
    # )

    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         "tf_prefix",
    #         default_value=EnvironmentVariable("TF_PREFIX", default_value=""),
    #         description="Add prefix to the all robot's links & joints.",
    #     )
    # )

    declared_arguments.append(
        DeclareLaunchArgument(
            "mock_hardware",
            default_value=EnvironmentVariable("MOCK_HARDWARE", default_value="false"),
            description="Mock the hardware for testing purposes.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "disable_cell",
            default_value=EnvironmentVariable("DISABLE_CELL", default_value="false"),
            description="Exclude the cell from description.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "disable_cell_collision",
            default_value=EnvironmentVariable(
                "DISABLE_CELL_COLLISION", default_value="false"
            ),
            description="Exclude the cell collision from description.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value=EnvironmentVariable("LAUNCH_RVIZ", default_value="true"),
            description="Launch RViz for robot state visualization & MoveIt2 manual control.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "disable_cameras",
            default_value=EnvironmentVariable("DISABLE_CAMERAS", default_value="false"),
            description="Do not launch the cameras drivers.",
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
