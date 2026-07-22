from launch import LaunchDescription, LaunchContext, LaunchDescriptionEntity
from launch.actions import IncludeLaunchDescription, OpaqueFunction

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([OpaqueFunction(function=launch_setup)])


def launch_setup(context: LaunchContext) -> list[LaunchDescriptionEntity]:
    launch_args = {
        "tf_prefix": LaunchConfiguration("tf_prefix", default=""),
        "mock_hardware": LaunchConfiguration("mock_hardware", default="false"),
    }

    depthai_cameras_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("aegis_control"),
                    "launch",
                    "depthai_cameras_driver.launch.py",
                ]
            )
        ),
        launch_arguments=launch_args.items(),
    )

    pylon_cameras_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("aegis_control"),
                    "launch",
                    "pylon_cameras_driver.launch.py",
                ]
            )
        ),
        launch_arguments=launch_args.items(),
    )

    return [
        depthai_cameras_driver,
        pylon_cameras_driver,
    ]
