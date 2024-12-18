from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:

    launch_arguments = {
        "namespace": LaunchConfiguration("namespace", default=""),
        "mock_hardware": LaunchConfiguration("mock_hardware", default="false"),
    }

    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("aegis_control"),
                    "launch",
                    "robot_description.launch.py",
                ]
            )
        ),
        launch_arguments=launch_arguments.items(),
    )

    ur_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("aegis_control"), "launch", "ur_driver.launch.py"]
            )
        ),
        launch_arguments=launch_arguments.items(),
    )

    return LaunchDescription(
        [
            robot_description,
            # ur_driver,
        ]
    )
