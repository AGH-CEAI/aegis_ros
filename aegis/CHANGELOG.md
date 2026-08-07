# Changelog

All notable changes to the `aegis` package will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
### Added
* [PR-132](https://github.com/AGH-CEAI/aegis_ros/pull/130) - gRPC control over [wled_ros_driver](https://github.com/AGH-CEAI/wled_ros_driver).
* [PR-129](https://github.com/AGH-CEAI/aegis_ros/pull/129) - Added `wled_ros_driver` repository
* [PR-128](https://github.com/AGH-CEAI/aegis_ros/pull/128) - Added a remote setup guide for connecting to the robot station over the network.

### Changed
### Deprecated
### Removed
### Fixed

* [PR-134](https://github.com/AGH-CEAI/aegis_ros/pull/134) - Updated the `scene_objects_manager` repository (updated in PR #115).

### Security

## [v202603091730]

### Changed

* [PR-114](https://github.com/AGH-CEAI/aegis_ros/pull/114) - Updated docs before release.
* [PR-106](https://github.com/AGH-CEAI/aegis_ros/pull/106) - Updated `aegis.repos`: `pylon_ros_camera` to `a739e4c303`.
* [PR-90](https://github.com/AGH-CEAI/aegis_ros/pull/90) - Updated `aegis.repos`: `robotiq_hande_driver` to `v0.2.2-humble`.
* [PR-75](https://github.com/AGH-CEAI/aegis_ros/pull/75) - Changed `pymoveit2` upstream repo to a [custom fork](https://github.com/macmacal/pymoveit2).
* [PR-68](https://github.com/AGH-CEAI/aegis_ros/pull/68) - Updated repos for new robotiq_hande_driver fix.

## [v202509011041]

### Fixed

* [PR-58](https://github.com/AGH-CEAI/aegis_ros/pull/58) - Fixed changelogs and registered project repository in the [Zenodo](https://zenodo.org/).

## [v202508271325]

### Added

* [PR-43](https://github.com/AGH-CEAI/aegis_ros/pull/43) - Integration with [`pylon-ros-camera`](https://github.com/basler/pylon-ros-camera) package.
* [PR-37](https://github.com/AGH-CEAI/aegis_ros/pull/37) - Integration with [`pymoveit2`](https://github.com/AndrejOrsula/pymoveit2) package.
* [PR-25](https://github.com/AGH-CEAI/aegis_ros/pull/25) - Integration with [`robotiq_hande_driver`](https://github.com/AGH-CEAI/robotiq_hande_driver) package.
* [PR-7](https://github.com/AGH-CEAI/aegis_ros/pull/7) - Initial version of the Aegis ROS 2 packages.

### Changed

* [PR-49](https://github.com/AGH-CEAI/aegis_ros/pull/27) - Updated `robotiq_hande_description` and `robotiq_hande_driver` packages.
* [PR-25](https://github.com/AGH-CEAI/aegis_ros/pull/25) - Updated `robotiq_hande_description` package.

### Fixed

* [PR-18](https://github.com/AGH-CEAI/aegis_ros/pull/18) - Updated the `ros2_net_ft_driver` to fix missing `curlpp-dev` dependency.
