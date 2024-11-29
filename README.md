# aegis_ros

A complete suite of ROS 2 packages for the Aegis UR5e cobot station.

[![pre-commit](https://img.shields.io/badge/pre--commit-enabled-brightgreen?logo=pre-commit)](https://github.com/pre-commit/pre-commit)

---

## List of packages

* [aegis_bringup](./aegis_bringup/README.md) - the main launch file with all hardware-dependend configuration files.
* [aegis_description](./aegis_description/README.md) - the description of the Aegis robot station.
* [aegis_moveit_config](./aegis_moveit_config/README.md) - the collection of configuration files to run the [MoveIt2](https://moveit.picknik.ai/main/index.html).

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
toolbox create --image localhost/ceai/aegis_dev
toolbox enter aegis_dev
```

### Build
```bash
cd ~/ceai_ws
source /opt/ros/humble/setup.bash
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
