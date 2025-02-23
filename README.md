# aegis_ros

A complete suite of ROS 2 packages for the Aegis UR5e cobot station.

[![pre-commit](https://img.shields.io/badge/pre--commit-enabled-brightgreen?logo=pre-commit)](https://github.com/pre-commit/pre-commit)

<p align="center">
    <img src="./.docs/aegis_station.png" alt="Aegis cobot station preview" width="640"/>
</p>

---

## List of packages

* `aegis`: The meta-package for referencing all project dependencies.
* [aegis_control](./aegis_control/README.md): Launch files dedicated to the hardware drivers based on the [ros2_control](https://control.ros.org/humble/doc/getting_started/getting_started.html) framework.
* [aegis_bringup](./aegis_bringup/README.md): The main launch file.
* [aegis_description](./aegis_description/README.md): The description and configuration files of the Aegis robot station.
* [aegis_moveit_config](./aegis_moveit_config/README.md): The configuration to run the [MoveIt 2](https://moveit.picknik.ai/main/index.html) framework.

---

## Quick start

### Create workspace

```bash
mkdir -p ~/ceai_ws
cd ~/ceai_ws
git clone -b humble-devel https://github.com/AGH-CEAI/aegis_ros.git src/aegis_ros
```

### Containers

> [!TIP]
> Check the [aegis_docker](https://github.com/AGH-CEAI/aegis_docker) repository for the Dockerfile.

#### Docker
```bash
cd ~/ceai_ws/src/aegis_docker
docker build . -t ceai/aegis_dev:latest
#TODO add more examples
```

#### Podman & Toolbx
```bash
cd ~/ceai_ws/src/aegis_docker
podman build . -t ceai/aegis_dev:latest
toolbox create --image localhost/ceai/aegis_dev:latest
toolbox enter aegis_dev-latest
```

### Resolving dependencies and build
```bash
source /opt/ros/humble/setup.bash
rosdep init
```
```bash
cd ~/ceai_ws
vcs import src < src/aegis_ros/aegis/aegis.repos
rosdep update --rosdistro $ROS_DISTRO
rosdep install --from-paths src -y -i
colcon build --symlink-install
source ./install/local_setup.bash
```

### Run

See the [aegis_bringup](./aegis_bringup/README.md) package.


---
## Development notes

This project uses various tools for aiding the quality of the source code. Currently most of them are executed by the `pre-commit`. Please make sure to enable its hooks:

```bash
pre-commit install
```

---
## License
This repository is licensed under the Apache 2.0, see LICENSE for details.
