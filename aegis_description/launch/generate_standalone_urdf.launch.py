import re
import shutil
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import FindExecutable


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("save_dir", default_value="~/ceai_ws/aegis_urdf"),
            DeclareLaunchArgument("tf_prefix", default_value=""),
            DeclareLaunchArgument("mock_hardware", default_value="false"),
            DeclareLaunchArgument("disable_cell", default_value="false"),
            DeclareLaunchArgument("disable_cell_collision", default_value="false"),
            OpaqueFunction(function=evaluate_robot_description),
        ]
    )


def evaluate_robot_description(context, *args, **kwargs) -> list:
    save_dir = (
        Path(LaunchConfiguration("save_dir").perform(context)).expanduser().resolve()
    )
    save_dir.mkdir(exist_ok=True)
    urdf_path = save_dir / "aegis.urdf"

    tf_prefix = LaunchConfiguration("tf_prefix")
    mock_hardware = LaunchConfiguration("mock_hardware")
    disable_cell = LaunchConfiguration("disable_cell")
    disable_cell_collision = LaunchConfiguration("disable_cell_collision")

    robot_description_str = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("aegis_description"), "urdf", "aegis.urdf.xacro"]
            ),
            " ",
            "tf_prefix:=",
            tf_prefix,
            " ",
            "mock_hardware:=",
            mock_hardware,
            " ",
            "disable_cell:=",
            disable_cell,
            " ",
            "disable_cell_collision:=",
            disable_cell_collision,
        ]
    ).perform(context)
    urdf_str, files_paths = extract_files_paths(robot_description_str)
    print(f"> Found {len(files_paths)} 3D models to be copied")

    for file in files_paths:
        shutil.copy(src=file, dst=save_dir / file.name)
    print(f"> Copied all 3D model files files to '{save_dir}'")

    urdf_path.write_bytes(urdf_str.encode("utf-8"))
    print(f"> URDF generated to '{urdf_path}'")

    return []


def extract_files_paths(urdf_str: str) -> tuple[str, set[Path]]:
    # Regex to get: 1) package name 2) rel. 3D model path 3) 3D model filename
    pattern = r"filename=\"package://(\w+)/?(.+/)?([^/]+\.(?:stl|dae|obj|3ds))\""
    matches = re.findall(pattern, urdf_str)

    files_paths = set()
    for pkg_name, rel_model_path, filename in matches:
        package_path = Path(get_package_share_directory(pkg_name))
        file_path = package_path / rel_model_path / filename

        files_paths.add(file_path)
        urdf_str = urdf_str.replace(f"package://{pkg_name}/{rel_model_path}", "./")
    return urdf_str, files_paths
