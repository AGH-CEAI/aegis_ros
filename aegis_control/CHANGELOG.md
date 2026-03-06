# Changelog

All notable changes to the `aegis_control` package will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

* [PR-106](https://github.com/AGH-CEAI/aegis_ros/pull/106) - Added script for automatic Pylon cameras ROI setup.
* [PR-65](https://github.com/AGH-CEAI/aegis_ros/pull/65) - Added static transformation nodes for cameras from calibration.
* [PR-63](https://github.com/AGH-CEAI/aegis_ros/pull/63) - Automatic call of the "Play" service of the `ur_dashboard` node.

### Changed

* [PR-112](https://github.com/AGH-CEAI/aegis_ros/pull/112) - Cropped scene camera image output.
* [PR-108](https://github.com/AGH-CEAI/aegis_ros/pull/108) - Reduced DepthAI pipeline for scene camera to RGB-only.
* [PR-106](https://github.com/AGH-CEAI/aegis_ros/pull/106) - Pylon cameras: Fixed exposure time, enabled automatic gain adjustment for fixed brightness value.
* [PR-106](https://github.com/AGH-CEAI/aegis_ros/pull/106) - Pylon cameras: Enabled automatic white balance.
* [PR-68](https://github.com/AGH-CEAI/aegis_ros/pull/68) - Changed gripper_action_controller's action monitor rate from `20` to `10` Hz.
* [PR-60](https://github.com/AGH-CEAI/aegis_ros/pull/60) - Enabled all UR driver controllers.

### Deprecated
### Removed
### Fixed
### Security

## [v202509011041]

### Fixed

* [PR-58](https://github.com/AGH-CEAI/aegis_ros/pull/58) - Fixed changelogs.

## [v202508271325]

### Added

* [PR-54](https://github.com/AGH-CEAI/aegis_ros/pull/54) - Process tool cameras' images (Basler).
* [PR-44](https://github.com/AGH-CEAI/aegis_ros/pull/44) - Integration with Luxonis tool camera.
* [PR-43](https://github.com/AGH-CEAI/aegis_ros/pull/43) - Integration with Balser tool cameras.
* [PR-25](https://github.com/AGH-CEAI/aegis_ros/pull/25) - Integration with `robotiq_hande_driver` package.
* [PR-30](https://github.com/AGH-CEAI/aegis_ros/pull/30) - Implemented YOLOv5 model for spatial detection.
* [PR-24](https://github.com/AGH-CEAI/aegis_ros/pull/24) - Implemented point cloud support.
* [PR-21](https://github.com/AGH-CEAI/aegis_ros/pull/21) - Initial version of the DepthAI driver with support for the OAK-D Pro camera.
* [PR-9](https://github.com/AGH-CEAI/aegis_ros/pull/9) - Initial version of the `aegis_control` package.

### Changed

* [PR-53](https://github.com/AGH-CEAI/aegis_ros/pull/53) - Renamed `cam_tool` to `cam_tool_front`.
* [PR-49](https://github.com/AGH-CEAI/aegis_ros/pull/27) - Disabled the tool communication in the `ur_driver` in favor of `robotiq_hande_driver`'s `SocatManager`.
* [PR-27](https://github.com/AGH-CEAI/aegis_ros/pull/27) - DepthAI nodes can be disabled with `mock_hardware` flag.
* [PR-20] (https://github.com/AGH-CEAI/aegis_ros/pull/20) - Automatic initialization of the virtual serial port.
* [PR-17](https://github.com/AGH-CEAI/aegis_ros/pull/17) - Added launch file for the ros2_net_ft_driver.
* [PR-16](https://github.com/AGH-CEAI/aegis_ros/pull/16) - Smoother controller_manager node config composition.
