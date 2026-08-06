# Changelog

All notable changes to the `aegis_grpc` package will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
* [PR-132](https://github.com/AGH-CEAI/aegis_ros/pull/130) - gRPC control over wled service.

### Changed
* [PR-130](https://github.com/AGH-CEAI/aegis_ros/pull/130) - New ruff formatting.
* [PR-125](https://github.com/AGH-CEAI/aegis_ros/pull/125) - Client: Reordered the joints to match simulation joints order.
* [PR-125](https://github.com/AGH-CEAI/aegis_ros/pull/125) - Client: Changed cameras names and introduced StrEnums for all literals.
* [PR-119](https://github.com/AGH-CEAI/aegis_ros/pull/119) - Server: parametrized topics subs buffer size (and changed default from `10` to `1`).
### Deprecated
### Removed
### Fixed
### Security

## [v202603091730]

### Added
* [PR-109](https://github.com/AGH-CEAI/aegis_ros/pull/109) - Client: Enum with indexes of all joints.
* [PR-104](https://github.com/AGH-CEAI/aegis_ros/pull/104) - Server: Omit MoveIt2 call if the target is close to current pose.
* [PR-103](https://github.com/AGH-CEAI/aegis_ros/pull/103) - Server: Adds servo idling with zero commands.
* [PR-98](https://github.com/AGH-CEAI/aegis_ros/pull/98) - Added gRPC-based camera image reading.
* [PR-97](https://github.com/AGH-CEAI/aegis_ros/pull/97) - Added gRPC func: MoveIt2 servo start & stop service.
* [PR-96](https://github.com/AGH-CEAI/aegis_ros/pull/96) - Added gRPC func: ros2_control controller switching.
* [PR-92](https://github.com/AGH-CEAI/aegis_ros/pull/92) - Added Python client installation script.
* [PR-88](https://github.com/AGH-CEAI/aegis_ros/pull/88) - Added gRPC robot motion commands with MoveIt.
* [PR-88](https://github.com/AGH-CEAI/aegis_ros/pull/88) - MVP of the gRPC Python client.
* [PR-82](https://github.com/AGH-CEAI/aegis_ros/pull/82) - MVP of the gRPC-ROS server.

### Changed

* [PR-114](https://github.com/AGH-CEAI/aegis_ros/pull/114) - Updated docs before release.
* [PR-103](https://github.com/AGH-CEAI/aegis_ros/pull/103) - Server: Changed `assert()` in `FillProtoImage()` to `if` which loggs an error message.
* [PR-103](https://github.com/AGH-CEAI/aegis_ros/pull/103) - Client: The Python client `aegis_grpc.py` will now log every message with the prefix `[aegis_robot_client]`.
* [PR-102](https://github.com/AGH-CEAI/aegis_ros/pull/102) - Server: The end effector's TCP pose is now taken from the tf2 transform instead of a given topic.
