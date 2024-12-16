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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:

    namespace = LaunchConfiguration("namespace")
    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value=EnvironmentVariable("ROBOT_NAMESPACE", default_value=""),
        description="Add namespace to all launched nodes.",
    )

    mock_hardware = LaunchConfiguration("mock_hardware")
    declare_mock_hardware_arg = DeclareLaunchArgument(
        "mock_hardware",
        default_value=EnvironmentVariable("MOCK_HARDWARE", default_value="false"),
        description="Mock the hardware for testing purposes.",
    )

    # ur_driver_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         PathJoinSubstitution(
    #             [FindPackageShare("ur_robot_driver"), "launch", "ur_control.launch.py"]
    #         )
    #     ),
    #     launch_arguments={
    #         "namespace": namespace,
    #         "use_fake_hardware": fake_hardware,
    #         "ur_type": "ur5e",
    #         "robot_ip": "aegis_ur",
    #         "launch_rviz": "false",
    #     }.items(),
    # )

    drivers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("aegis_control"), "launch", "start_drivers.launch.py"]
            )
        ),
        launch_arguments={
            "namespace": namespace,
            "mock_hardware": mock_hardware,
        }.items(),
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
        launch_arguments={"launch_rviz": "true"}.items(),
    )

    return LaunchDescription(
        [
            declare_namespace_arg,
            declare_mock_hardware_arg,
            drivers_launch,
            moveit_launch,
        ]
    )
