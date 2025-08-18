from setuptools import find_packages, setup

package_name = "aegis_utils"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (
            f"share/{package_name}/config",
            [
                "config/cam_scene.yaml",
                "config/cam_tool_front.yaml",
                "config/cam_tool_right.yaml",
                "config/cam_tool_left.yaml",
                "config/charuco_board.yaml",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Jakub Plachno",
    maintainer_email="sivral@proton.me",
    description="A package containing utility functions for the Aegis robot station",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "calib_data_collect = aegis_utils.calib_data_collect:main",
            "calib_intrinsics = aegis_utils.calib_intrinsics:main",
            "calib_extrinsics = aegis_utils.calib_extrinsics:main",
        ],
    },
)
