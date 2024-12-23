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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:

    namespace = LaunchConfiguration("namespace")
    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value=EnvironmentVariable("ROBOT_NAMESPACE", default_value=""),
        description="Add namespace to all launched nodes.",
    )

    tf_prefix = LaunchConfiguration("tf_prefix")
    declare_tf_prefix_arg = DeclareLaunchArgument(
        "namespace",
        default_value=EnvironmentVariable("TF_PREFIX", default_value=""),
        description="Add prefix to the all robot's links & joints.",
    )

    mock_hardware = LaunchConfiguration("mock_hardware")
    declare_mock_hardware_arg = DeclareLaunchArgument(
        "mock_hardware",
        default_value=EnvironmentVariable("MOCK_HARDWARE", default_value="false"),
        description="Mock the hardware for testing purposes.",
    )

    launch_rviz = LaunchConfiguration("launch_rviz")
    declare_launch_rviz_arg = DeclareLaunchArgument(
        "launch_rviz",
        default_value=EnvironmentVariable("LAUNCH_RVIZ", default_value="true"),
        description="Launch RViz for robot state visualization & MoveIt2 manual control.",
    )

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
        launch_arguments={
            "tf_prefix": tf_prefix,
            "mock_hardware": mock_hardware,
        }.items(),
    )
    robot_description_launch = GroupAction(
        actions=[PushRosNamespace(namespace), robot_description_launch]
    )

    drivers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("aegis_control"), "launch", "start_drivers.launch.py"]
            )
        ),
        launch_arguments={
            "tf_prefix": tf_prefix,
            "mock_hardware": mock_hardware,
        }.items(),
    )
    drivers_launch = GroupAction(actions=[PushRosNamespace(namespace), drivers_launch])

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
        launch_arguments={
            "mock_hardware": mock_hardware,
            "launch_rviz": launch_rviz,
        }.items(),
    )
    moveit_launch = GroupAction(actions=[PushRosNamespace(namespace), moveit_launch])

    return LaunchDescription(
        [
            declare_namespace_arg,
            declare_tf_prefix_arg,
            declare_mock_hardware_arg,
            declare_launch_rviz_arg,
            robot_description_launch,
            drivers_launch,
            moveit_launch,
        ]
    )
