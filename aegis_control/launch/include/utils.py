from typing import Optional
from launch import Condition, LaunchContext, LaunchDescriptionEntity
from launch.actions import IncludeLaunchDescription, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessStart
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


def get_node_from_include_launch_description(
    node_name: str, desc: IncludeLaunchDescription, context: LaunchContext
) -> Optional[LaunchDescriptionEntity]:
    """
    Searches for a specific Node launch configuration within an `IncludeLaunchDescription` action by its name.

    Args:
        node_name (str): The name of the ROS 2 node to search for.
        desc (IncludeLaunchDescription): The `IncludeLaunchDescription` action containing the launch description.
        context (LaunchContext): The launch context used to evaluate the launch description.

    Returns:
        Optional[LaunchDescriptionEntity]: The matching node entity if found, otherwise None.
    """
    launch_desc = desc.launch_description_source.get_launch_description(context)
    for entry in launch_desc.entities:
        if "launch_ros.actions.node.Node" in entry.describe():
            entry_node_name = entry._Node__node_name
            if node_name == entry_node_name:
                return entry
    return None


def launch_after(
    launch: LaunchDescriptionEntity,
    after: LaunchDescriptionEntity,
) -> LaunchDescriptionEntity:
    """
    Registers an event handler to start a launch action after another one has started.

    Args:
        launch (LaunchDescriptionEntity): The launch action to start after the specified action.
        after (LaunchDescriptionEntity): The launch action that must start first.

    Returns:
        LaunchDescriptionEntity: A RegisterEventHandler that triggers `launch` once `after` starts.
    """

    return RegisterEventHandler(
        OnProcessStart(
            target_action=after,
            on_start=[
                LogInfo(
                    msg=f"{after.describe()} started, spawning {launch.describe()}"
                ),
                launch,
            ],
        )
    )
