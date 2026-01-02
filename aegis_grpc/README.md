# aegis_grpc

This packages provides a bridge between ROS 2 and [gRPC](https://grpc.io) protocols, with custom [Protobuf](https://protobuf.dev) messages based on selected ones from [`control_msgs`](https://github.com/ros-controls/control_msgs/tree/humble), [`geometry_msgs`](https://github.com/ros2/common_interfaces/tree/humble/geometry_msgs) and [`sensors_mgs`](https://github.com/ros2/common_interfaces/tree/rolling/sensor_msgs) packages.

It aims to provide a form of "frequency clutch" between real-time domain and low-frequnecy decision making (e.g. an ANN inference in Python).

## Usage

### Launch

``` bash
ros2 launch aegis_grpc start_server.launch.py
# or
ros2 run aegis_grpc grpc_server
```

### Interaction
The server have enabled a "reflection" plugin, which allows to explore all services with external tool like [`grpcurl`](https://github.com/fullstorydev/grpcurl).

```bash
grpcurl -plaintext 127.0.0.1:50051 list
grpcurl -plaintext 127.0.0.1:50051 list proto_aegis_grpc.v1.RobotControlService
grpcurl -plaintext 127.0.0.1:50051 list proto_aegis_grpc.v1.RobotReadService
grpcurl -plaintext 127.0.0.1:50051 describe proto_aegis_grpc.v1.RobotReadService.GetAll
# or with tool from container
podman run --network=host docker.io/fullstorydev/grpcurl -plaintext 127.0.0.1:50051 list
```

Example call to `GetAll` result as a plain json:

```bash
grpcurl -plaintext 127.0.0.1:50051 \
  -d '{}' \
  127.0.0.1:50051 \
  proto_aegis_grpc.v1.RobotReadService.GetAll
```

## Messages architecture

Since this project aims to bridge communication between ROS and gRPC, the protobuf definitions tries to mimic default messages, services and actions from ROS.
The main difference is the lack of the header with timestamps - the data synchronization responsibility should not be forwarded outside the ROS ecosystem.

The server is split into 2 services defined in [`proto_aegis_grpc.v1.robot_srvs`](./proto_aegis_grpc/v1/robot_srvs.proto): `RobotReadService` and `RobotControlService`.

### Reading data with `RobotReadService`

The "ROS-getters" are implemented in the [`RobotReadServiceImpl`](./include/aegis_grpc/robot_read_service.hpp) class as the following services:

| Name                                                 | Desc.                      | Impl. | gRPC Request            | gRPC Response                                                                           |
| ---------------------------------------------------- | -------------------------- | ----- | ----------------------- | --------------------------------------------------------------------------------------- |
| `proto_aegis_grpc.v1.RobotReadService.GetAll`        | Get the all available data. | ✅     | `google.protobuf.Empty` | [`proto_aegis_grpc.v1.robot_srvs.RobotState`](./proto_aegis_grpc/v1/robot_srvs.proto)   |
| `proto_aegis_grpc.v1.RobotReadService.GetJointState` | Get the joints' states.    | ✅     | `google.protobuf.Empty` | [`proto_aegis_grpc.v1.sensor_msgs.JointState`](./proto_aegis_grpc/v1/sensor_msgs.proto) |
| `proto_aegis_grpc.v1.RobotReadService.GetTCPPose`    | Get the TCP pose.          | ✅     | `google.protobuf.Empty` | [`proto_aegis_grpc.v1.geometry_msgs.Pose`](./proto_aegis_grpc/v1/geometry_msgs.proto)   |
| `proto_aegis_grpc.v1.RobotReadService.GetWrench`     | Read force/torque sensor.  | ✅     | `google.protobuf.Empty` | [`proto_aegis_grpc.v1.geometry_msgs.Wrench`](./proto_aegis_grpc/v1/geometry_msgs.proto) |

You can always list the above services them with the `grpcurl`:
```bash
grpcurl -plaintext 127.0.0.1:50051 list proto_aegis_grpc.v1.RobotReadService
```

### Controlling the robot with `RobotControlService`

The control bridge for MoveIt2 servo and planners are implemented in the [`RobotControlServiceImpl`](./include/aegis_grpc/robot_control_service.hpp) class as the following services:

| Name                                                     | Desc.                                                           | Impl. | gRPC Request                                                                                      | gRPC Response                                                                              |
| -------------------------------------------------------- | --------------------------------------------------------------- | ----- | ------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------ |
| `proto_aegis_grpc.v1.RobotControlService.GotoJoints`     | Plan & execute the MoveIt2 trajectory to a given joints target. | ❌     | [`proto_aegis_grpc.v1.sensor_msgs.JointState`](./proto_aegis_grpc/v1/sensor_msgs.proto)           | [`proto_aegis_grpc.v1.robot_srvs.TriggerResponse`](./proto_aegis_grpc/v1/robot_srvs.proto) |
| `proto_aegis_grpc.v1.RobotControlService.GotoPose`       | Plan & execute the MoveIt2 trajectory to a given pose target.   | ❌     | [`proto_aegis_grpc.v1.geometry_msgs.Pose`](./proto_aegis_grpc/v1/geometry_msgs.proto)             | [`proto_aegis_grpc.v1.robot_srvs.TriggerResponse`](./proto_aegis_grpc/v1/robot_srvs.proto) |
| `proto_aegis_grpc.v1.RobotControlService.GriperClose`    | Closes the gripper.                                             | ✅     | `google.protobuf.Empty`                                                                           | [`proto_aegis_grpc.v1.robot_srvs.TriggerResponse`](./proto_aegis_grpc/v1/robot_srvs.proto) |
| `proto_aegis_grpc.v1.RobotControlService.GriperOpen`     | Opens the gripper.                                              | ✅     | `google.protobuf.Empty`                                                                           | [`proto_aegis_grpc.v1.robot_srvs.TriggerResponse`](./proto_aegis_grpc/v1/robot_srvs.proto) |
| `proto_aegis_grpc.v1.RobotControlService.GriperSetWidth` | Set the gripper width.                                          | ✅     | [`proto_aegis_grpc.v1.robot_srvs.GripperSetWidthRequest`](./proto_aegis_grpc/v1/robot_srvs.proto) | [`proto_aegis_grpc.v1.robot_srvs.TriggerResponse`](./proto_aegis_grpc/v1/robot_srvs.proto) |
| `proto_aegis_grpc.v1.RobotControlService.ServoJoint`     | Servoing (w.r.t. joints).                                       | ✅     | [`proto_aegis_grpc.v1.control_msgs.JointJog`](./proto_aegis_grpc/v1/control_msgs.proto)           | `google.protobuf.Empty`                                                                    |
| `proto_aegis_grpc.v1.RobotControlService.ServoTCP`       | Servoing (w.r.t. TCP).                                          | ✅     | [`proto_aegis_grpc.v1.geometry_msgs.Twist`](./proto_aegis_grpc/v1/geometry_msgs.proto)            | `google.protobuf.Empty`                                                                    |

You can always list the above services them with the `grpcurl`:
```bash
grpcurl -plaintext 127.0.0.1:50051 list proto_aegis_grpc.v1.RobotControlService
```

### All messages

| gRPC message                                                                                      | ROS 2 interface                                                                                                                            |
| ------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------ |
| [`proto_aegis_grpc.v1.control_msgs.JointJog`](./proto_aegis_grpc/v1/control_msgs.proto)           | [`control_msgs/msg/JointJog`](https://github.com/ros-controls/control_msgs/blob/humble/control_msgs/msg/JointJog.msg)                      |
| [`proto_aegis_grpc.v1.geometry_msgs.Vector3`](./proto_aegis_grpc/v1/geometry_msgs.proto)          | [`geometry_msgs/msg/Vector3`](https://github.com/ros2/common_interfaces/blob/humble/geometry_msgs/msg/Vector3.msg)                         |
| [`proto_aegis_grpc.v1.geometry_msgs.Point`](./proto_aegis_grpc/v1/geometry_msgs.proto)            | [`geometry_msgs/msg/Point`](https://github.com/ros2/common_interfaces/blob/humble/geometry_msgs/msg/Point.msg)                             |
| [`proto_aegis_grpc.v1.geometry_msgs.Quaternion`](./proto_aegis_grpc/v1/geometry_msgs.proto)       | [`geometry_msgs/msg/Quaternion`](https://github.com/ros2/common_interfaces/blob/humble/geometry_msgs/msg/Quaternion.msg)                   |
| [`proto_aegis_grpc.v1.geometry_msgs.Pose`](./proto_aegis_grpc/v1/geometry_msgs.proto)             | [`geometry_msgs/msg/Pose`](https://github.com/ros2/common_interfaces/blob/humble/geometry_msgs/msg/Pose.msg)                               |
| [`proto_aegis_grpc.v1.geometry_msgs.Wrench`](./proto_aegis_grpc/v1/geometry_msgs.proto)           | [`geometry_msgs/msg/Wrench`](https://github.com/ros2/common_interfaces/blob/humble/geometry_msgs/msg/Wrench.msg)                           |
| [`proto_aegis_grpc.v1.geometry_msgs.Twist`](./proto_aegis_grpc/v1/geometry_msgs.proto)            | [`geometry_msgs/msg/Twist`](https://github.com/ros2/common_interfaces/blob/humble/geometry_msgs/msg/Twist.msg)                             |
| [`proto_aegis_grpc.v1.robot_srvs.RobotState`](./proto_aegis_grpc/v1/robot_srvs.proto)             | -                                                                                                                                          |
| [`proto_aegis_grpc.v1.robot_srvs.TriggerResponse`](./proto_aegis_grpc/v1/robot_srvs.proto)        | [`std_srvs/srv/Trigger`](https://github.com/ros2/common_interfaces/blob/humble/std_srvs/srv/Trigger.srv)                                   |
| [`proto_aegis_grpc.v1.robot_srvs.GripperSetWidthRequest`](./proto_aegis_grpc/v1/robot_srvs.proto) | [`control_msgs/action/GripperCommand`](https://github.com/ros-controls/control_msgs/blob/humble/control_msgs/action/GripperCommand.action) |

---

## Development notes

You can simulate data on ROS topics with these commands:
```bash
ros2 topic pub /tcp_pose geometry_msgs/msg/PoseStamped "{header: auto, pose: {position: {x: 1.0, y: 2.0, z: 3.0}, orientation: {x: 4.0, y: 5.0, z: 6.0, w: 7.0}}}" --once
ros2 topic pub /joint_states sensor_msgs/msg/JointState "{header: auto, name: ['joint1','joint2'], position: [0.0, 1.0], velocity: [2.0, 3.0], effort: [4.0, 5.0]}" --once
ros2 topic pub /wrench geometry_msgs/msg/WrenchStamped "{header: auto, wrench: {force: {x: 1.0, y: 2.0, z: 3.0}, torque: {x: 4.0, y: 5.0, z: 6.0}}}" --once
```

---

## Credits
* Heavily inspired by [an example](https://github.com/tasada038/ros2_grpc_server) by [Takumi Asada @tasada038](https://github.com/tasada038).
  * Modified version with polished rosdep management can be accessed [here](https://github.com/macmacal/ros2_grpc_server).
