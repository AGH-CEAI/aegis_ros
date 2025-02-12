from launch import LaunchDescription

# Bypassing the launch system to access local import
import os
import sys

run_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(run_path)
from include.utils import controllers_spawner  # noqa E402


def generate_launch_description():
    controllers_active = [
        "net_ft_sensor_broadcaster",
        "net_ft_diagnostic_broadcaster",
    ]

    return LaunchDescription([controllers_spawner(controllers_active)])
