from setuptools import find_packages, setup

package_name = "aegis_director"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/director.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Maciej Aleksandrowicz",
    maintainer_email="macal@agh.edu.pl",
    description="A package to coordinate the robot's procedural workflow",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["director = aegis_director.director:main"],
    },
)
