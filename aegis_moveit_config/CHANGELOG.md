# Changelog

All notable changes to the `aegis_moveit_config` package will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
### Changed
### Deprecated
### Removed
### Fixed
### Security

## [v202603091730]

### Added

* [PR-73](https://github.com/AGH-CEAI/aegis_ros/pull/73) - Enabled MoveIt2 servo.
* [PR-60](https://github.com/AGH-CEAI/aegis_ros/pull/60) - Added maximum acceleration limits config for Moveit planning.

### Changed

* [PR-114](https://github.com/AGH-CEAI/aegis_ros/pull/114) - Updated docs before release.
* [PR-112](https://github.com/AGH-CEAI/aegis_ros/pull/112) - Updated RViz visualization.
* [PR-106](https://github.com/AGH-CEAI/aegis_ros/pull/106) - Updated RViz config for better Pylon cameras preview.
* [PR-105](https://github.com/AGH-CEAI/aegis_ros/pull/105) - Calibrated servo configuration for Sim2Real experiments.
* [PR-77](https://github.com/AGH-CEAI/aegis_ros/pull/77) - Changed `ur_base` frame to the `world` frame (simulation simplification).
* [PR-75](https://github.com/AGH-CEAI/aegis_ros/pull/75) - Updated servo max linear, angular and joints velocities.
* [PR-71](https://github.com/AGH-CEAI/aegis_ros/pull/71) - Updated collision checking.
* [PR-68](https://github.com/AGH-CEAI/aegis_ros/pull/68) - Updated RViz config for proper QoS RGB camera config.
* [PR-60](https://github.com/AGH-CEAI/aegis_ros/pull/60) - Disabled MoveIt's Trajectory Execution Monitoring (TEM) - it is now possible to scale down robot speeds without trajectory errors.

## [v202509011041]

### Fixed

* [PR-58](https://github.com/AGH-CEAI/aegis_ros/pull/58) - Fixed changelogs.

## [v202508271325]

### Added

* [PR-53](https://github.com/AGH-CEAI/aegis_ros/pull/53) - Updated RViz visualization.
* [PR-44](https://github.com/AGH-CEAI/aegis_ros/pull/21) - RViz camera visualization for the Luxonis OAK-D SR tool camera.
* [PR-25](https://github.com/AGH-CEAI/aegis_ros/pull/25) - Integration with `robotiq_hande_driver` package.
* [PR-33](https://github.com/AGH-CEAI/aegis_ros/pull/33) - Enabled [OctoMap](https://octomap.github.io/) support.
* [PR-21](https://github.com/AGH-CEAI/aegis_ros/pull/21) - RViz camera visualization for the Luxonis OAK-D Pro camera.
* [PR-17](https://github.com/AGH-CEAI/aegis_ros/pull/17) - RViz wrench visualization for the F/T sensor.
* [PR-8](https://github.com/AGH-CEAI/aegis_ros/pull/8) - Added semantic description of the robot cell and the scene camera.
* [PR-7](https://github.com/AGH-CEAI/aegis_ros/pull/7) - Initial version of the Aegis ROS 2 packages.

### Changed

* [PR-42](https://github.com/AGH-CEAI/aegis_ros/pull/42) - Changed default pose from "zero" to "home"; moved OMPL config from launch to yaml.
* [PR-9](https://github.com/AGH-CEAI/aegis_ros/pull/9) - Launch and URDF files completely revamped.

### Fixed

* [PR-8](https://github.com/AGH-CEAI/aegis_ros/pull/8) - Handling of transformations.
