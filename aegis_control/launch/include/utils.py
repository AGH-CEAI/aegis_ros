from launch import Condition
from launch_ros.actions import Node


def str2bool(x: str) -> bool:
    return x.lower() in ("true")


def controllers_spawner(
    controllers: list[str],
    timeout_s: int = 10,
    active: bool = True,
    condition: Condition = None,
):
    inactive_flags = ["--inactive"] if not active else []
    return Node(
        package="controller_manager",
        executable="spawner",
        condition=condition,
        arguments=[
            "--controller-manager",
            "controller_manager",
            "--controller-manager-timeout",
            str(timeout_s),
        ]
        + inactive_flags
        + controllers,
    )
