# Bypassing the launch system to access local import
import os
import sys

from launch import LaunchContext, LaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

run_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(run_path)
from include.utils import controllers_spawner, str2bool


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([OpaqueFunction(function=launch_setup)])


def launch_setup(context: LaunchContext) -> list[Node]:
    mock_hardware = LaunchConfiguration("mock_hardware", default="false")
    mock_hardware_bool = str2bool(mock_hardware.perform(context))

    controllers_active = [
        "net_ft_sensor_broadcaster",
        "net_ft_diagnostic_broadcaster",
    ]

    if mock_hardware_bool:
        controllers_active.append("ft_mock_injector")

    return [controllers_spawner(controllers_active)]
