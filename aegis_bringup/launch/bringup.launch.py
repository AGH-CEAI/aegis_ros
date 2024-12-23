#!/usr/bin/env python3

# Copyright 2024 AGH Center of Excellence in Artificial Intelligence
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription, LaunchContext
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

    namespace = LaunchConfiguration("namespace")
    launch_args = {
        "tf_prefix": LaunchConfiguration("tf_prefix"),
        "mock_hardware": LaunchConfiguration("mock_hardware"),
        "launch_rviz": LaunchConfiguration("launch_rviz"),
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

    return set_namespace(
        namespace,
        [
            robot_description_launch,
            drivers_launch,
            moveit_launch,
        ],
    )


def set_namespace(namespace: LaunchConfiguration, description: list) -> list:
    return [
        GroupAction(actions=[PushRosNamespace(namespace), desc]) for desc in description
    ]


def generate_launch_description() -> LaunchDescription:

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "namespace",
            default_value=EnvironmentVariable("ROBOT_NAMESPACE", default_value=""),
            description="Add namespace to all launched nodes.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value=EnvironmentVariable("TF_PREFIX", default_value=""),
            description="Add prefix to the all robot's links & joints.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "mock_hardware",
            default_value=EnvironmentVariable("MOCK_HARDWARE", default_value="false"),
            description="Mock the hardware for testing purposes.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value=EnvironmentVariable("LAUNCH_RVIZ", default_value="true"),
            description="Launch RViz for robot state visualization & MoveIt2 manual control.",
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
