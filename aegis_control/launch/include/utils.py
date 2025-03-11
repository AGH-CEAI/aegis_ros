from launch import Condition
from launch_ros.actions import Node


def str2bool(x: str) -> bool:
    return x.lower() in ("true")


def controllers_spawner(
    controllers: list[str],
    timeout_s: int = 10,
    active: bool = True,
    condition: Condition = None,
) -> Node:
    """
    Spawn ros2_control controllers using spawner node from the `controller_manager` package.

    Args:
        controllers (List[str]): A list of controller names to be spawned.
        timeout_s (int, optional): Timeout in seconds for the controller manager. Defaults to 10.
        active (bool, optional): If False, the controllers will be spawned in an inactive state. Defaults to True.
        condition (Optional[Condition], optional): A launch condition to control execution. Defaults to None.

    Returns:
        Node: A ROS 2 launch `Node` action that executes the `spawner` command with the specified arguments.
    """
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
