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
git clone -b ros2 https://github.com/husarion/panther_ros.git src/panther_ros
```
### Containers
W.I.P.
```
#### Docker

#### Podman & Toolbx

### Build

### Launch Arguments
```

---
## Development notes

This project uses various tools for aiding the quality of the source code. Currently most of them are executed by the `pre-commit`. Please make sure to enable its hooks:

```bash
pre-commit install
```

---
## License
This repository is licensed under the Apache 2.0, see LICENSE for details.
