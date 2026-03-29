# Changelog

All notable changes to the `aegis_description` package will be documented in this file.

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

* [PR-95](https://github.com/AGH-CEAI/aegis_ros/pull/95) - Added standalone launch script to extract whole URDF model to a given directory.
* [PR-78](https://github.com/AGH-CEAI/aegis_ros/pull/78) - Added logic to allow selective inclusion of cell visual and collision geometry.

### Changed

* [PR-115](https://github.com/AGH-CEAI/aegis_ros/pull/115) - Changed the camera position.
* [PR-114](https://github.com/AGH-CEAI/aegis_ros/pull/114) - Updated docs before release.
* [PR-77](https://github.com/AGH-CEAI/aegis_ros/pull/77) - Changed `ur_base` frame to the `world` frame (simulation simplification).
* [PR-62](https://github.com/AGH-CEAI/aegis_ros/pull/62) - Replaced cell collision mesh with primitive shapes.
* [PR-60](https://github.com/AGH-CEAI/aegis_ros/pull/60) - Decreased the default velocity and acceleration scaling factor to 5%.
* [PR-57](https://github.com/AGH-CEAI/aegis_ros/pull/57) - Increased the position limit of the wrist 1 joint.

## [v202509011041]

### Fixed

* [PR-58](https://github.com/AGH-CEAI/aegis_ros/pull/58) - Fixed changelogs.

## [v202508271325]

### Added

* [PR-44](https://github.com/AGH-CEAI/aegis_ros/pull/44) - Added Luxonis OAK-D SR tool camera and its mount to the robot model.
* [PR-43](https://github.com/AGH-CEAI/aegis_ros/pull/43) - Added Basler ace tool cameras and their mounts to the robot model.
* [PR-8](https://github.com/AGH-CEAI/aegis_ros/pull/8) - Added the cell and the Luxonis OAK-D Pro scene camera to the robot model.
* [PR-7](https://github.com/AGH-CEAI/aegis_ros/pull/7) - Initial version of the Aegis ROS 2 packages.

### Changed

* [PR-72](https://github.com/AGH-CEAI/aegis_ros/pull/72) - Simplified Basler ace tool collision — replaced individual camera and lens collisions with a single mount collision. Renamed several links and joints, removed unnecessary ones.
* [PR-69](https://github.com/AGH-CEAI/aegis_ros/pull/69) - Changed cell collision.
* [PR-62](https://github.com/AGH-CEAI/aegis_ros/pull/62) - Changed cell collision.
* [PR-53](https://github.com/AGH-CEAI/aegis_ros/pull/53) - Renamed `cam_tool` to `cam_tool_front`.
* [PR-52](https://github.com/AGH-CEAI/aegis_ros/pull/52) - Renamed `ip_adress` to `socat_ip_address` and `port` to `socat_port`.
* [PR-49](https://github.com/AGH-CEAI/aegis_ros/pull/49) - Updated Hand-E gripper parameters.
* [PR-44](https://github.com/AGH-CEAI/aegis_ros/pull/44) - Updated adapter to sensor component and SCHUNK AXIA 80 F/T sensor.
* [PR-43](https://github.com/AGH-CEAI/aegis_ros/pull/43) - Updated adapter to sensor component.
* [PR-42](https://github.com/AGH-CEAI/aegis_ros/pull/42) - Updated joints limits.
* [PR-25](https://github.com/AGH-CEAI/aegis_ros/pull/25) - Updated Hand-E URDFs to incorporate ros2_control configuration.
* [PR-20] (https://github.com/AGH-CEAI/aegis_ros/pull/20) - Automatic initialization of the virtual serial port.
* [PR-17](https://github.com/AGH-CEAI/aegis_ros/pull/17) - Set default F/T sensor IP address.
* [PR-9](https://github.com/AGH-CEAI/aegis_ros/pull/9) - Launch and URDF files completely revamped.

### Fixed

* [PR-21](https://github.com/AGH-CEAI/aegis_ros/pull/21) - Switched the Luxonis OAK-D Pro camera description to the official Luxonis repository.
