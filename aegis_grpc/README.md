# aegis_grpc

This packages provides a bridge between ROS 2 and [gRPC](https://grpc.io) protocols, with custom [Protobuf](https://protobuf.dev) messages based on selected ones from [`control_msgs`](https://github.com/ros-controls/control_msgs/tree/humble), [`geometry_msgs`](https://github.com/ros2/common_interfaces/tree/humble/geometry_msgs) and [`sensors_mgs`](https://github.com/ros2/common_interfaces/tree/rolling/sensor_msgs) packages.

It aims to provide a form of "frequency clutch" between real-time domain and low-frequnecy decision making (e.g. an ANN inference in Python).

## Launch

``` bash
ros2 launch aegis_grpc start_server.launch.py
# or
ros2 run aegis_grpc grpc_server
```

## Messages architecture

Since this project aims to bridge communication between ROS and gRPC, the protobuf definitions tries to mimic default messages, services and actions from ROS.
The main difference is the lack of the header with timestamps - the data synchronization responsibility should not be forwarded.


### Read data

| Name       | Desc                      | Impl. | Type  | gRPC Def                                                                          | ROS Def                                                                                                              |
| ---------- | ------------------------- | ----- | ----- | --------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------- |
| Wrench     | Read force/torque sensor. | ❌     | `msg` | [`proto_aegis_grpc.v1.geometry_msgs.Wrench`](./proto/v1/geometry_msgs.proto)      | [`geometry_msgs/msg/Wrench`](https://github.com/ros2/common_interfaces/blob/humble/geometry_msgs/msg/Wrench.msg)     |
| Pose       | Get the TCP pose.         | ❌     | `msg` | [`proto_aegis_grpc.v1.geometry_msgs.Pose`](./proto/v1/geometry_msgs.proto)        | [`geometry_msgs/msg/Pose`](https://github.com/ros2/common_interfaces/blob/humble/geometry_msgs/msg/Pose.msg)         |
| JointState | Get the joints' states.   | ❌     | `msg` | [`proto_aegis_grpc.v1.sensor_msgs_msgs.JointState`](./proto/v1/sensor_msgs.proto) | [`sensor_msgs/msg/JointState`](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/JointState.msg) |


### Send data

| Name                     | Desc                                                            | Impl. | Type  | gRPC Def                                                                               | ROS Def                                                                                                                                    |
| ------------------------ | --------------------------------------------------------------- | ----- | ----- | -------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------ |
| GripperService/SetWidth  | Set the gripper width.                                          | ❌     | `rpc` | [`proto_aegis_grpc.v1.control_msgs.GripperCommand`](./proto/v1/control_msgs.proto)     | [`control_msgs/action/GripperCommand`](https://github.com/ros-controls/control_msgs/blob/humble/control_msgs/action/GripperCommand.action) |
| Twist                    | Servoing (w.r.t. TCP).                                          | ❌     | `msg` | [`proto_aegis_grpc.v1.geometry_msgs.Twist`](./proto/v1/geometry_msgs.proto)            | [`geometry_msgs/msg/TwistPose`](https://github.com/ros2/common_interfaces/blob/humble/geometry_msgs/msg/Twist.msg)                         |
| JointJog                 | Servoing (w.r.t. joints).                                       | ❌     | `msg` | [`proto_aegis_grpc.v1.control_msgs.JointJog`](./proto/v1/control_msgs.proto)           | [`control_msgs/msg/JointJog`](https://github.com/ros-controls/control_msgs/blob/humble/control_msgs/msg/JointJog.msg)                      |
| MoveitService/GotoPose   | Plan & execute the MoveIt2 trajectory to a given pose target.   | ❌     | `rpc` | [`proto_aegis_grpc.v1.moveit_facade.MoveToPoseTarget`](./proto/v1/control_msgs.proto)  | -                                                                                                                                          |
| MoveitService/GotoJoints | Plan & execute the MoveIt2 trajectory to a given joints target. | ❌     | `rpc` | [`proto_aegis_grpc.v1.moveit_facade.MoveToJointTarget`](./proto/v1/control_msgs.proto) | -                                                                                                                                          |


### All messages

| gRPC Def                                                                                | Type      | Name               | ROS Def                                                                                                                                    |
| --------------------------------------------------------------------------------------- | --------- | ------------------ | ------------------------------------------------------------------------------------------------------------------------------------------ |
| [`proto_aegis_grpc.v1.control_msgs.GripperService`](./proto/v1/control_msgs.proto)      | `service` | GripperService     | [`control_msgs/action/GripperCommand`](https://github.com/ros-controls/control_msgs/blob/humble/control_msgs/action/GripperCommand.action) |
| [`proto_aegis_grpc.v1.control_msgs.SetWidthRequest`](./proto/v1/control_msgs.proto)     | `msg`     | GripperStatus      | -                                                                                                                                          |
| [`proto_aegis_grpc.v1.control_msgs.SetWidthResponse`](./proto/v1/control_msgs.proto)    | `msg`     | SetWidthResponse   | -                                                                                                                                          |
| [`proto_aegis_grpc.v1.control_msgs.JointJog`](./proto/v1/control_msgs.proto)            | `msg`     | JointJog           | [`control_msgs/msg/JointJog`](https://github.com/ros-controls/control_msgs/blob/humble/control_msgs/msg/JointJog.msg)                      |
| [`proto_aegis_grpc.v1.geometry_msgs.Point`](./proto/v1/geometry_msgs.proto)             | `msg`     | Point              | [`geometry_msgs/msg/Point`](https://github.com/ros2/common_interfaces/blob/humble/geometry_msgs/msg/Point.msg)                             |
| [`proto_aegis_grpc.v1.geometry_msgs.Pose`](./proto/v1/geometry_msgs.proto)              | `msg`     | Pose               | [`geometry_msgs/msg/Pose`](https://github.com/ros2/common_interfaces/blob/humble/geometry_msgs/msg/Pose.msg)                               |
| [`proto_aegis_grpc.v1.geometry_msgs.Quaternion`](./proto/v1/geometry_msgs.proto)        | `msg`     | Quaternion         | [`geometry_msgs/msg/Pose`](https://github.com/ros2/common_interfaces/blob/humble/geometry_msgs/msg/Quaternion.msg)                         |
| [`proto_aegis_grpc.v1.geometry_msgs.Twist`](./proto/v1/geometry_msgs.proto)             | `msg`     | Twist              | [`geometry_msgs/msg/TwistPose`](https://github.com/ros2/common_interfaces/blob/humble/geometry_msgs/msg/Twist.msg)                         |
| [`proto_aegis_grpc.v1.geometry_msgs.Vector3`](./proto/v1/geometry_msgs.proto)           | `msg`     | Vector3            | [`geometry_msgs/msg/Vector3`](https://github.com/ros2/common_interfaces/blob/humble/geometry_msgs/msg/Vector3.msg)                         |
| [`proto_aegis_grpc.v1.geometry_msgs.Wrench`](./proto/v1/geometry_msgs.proto)            | `msg`     | Wrench             | [`geometry_msgs/msg/Wrench`](https://github.com/ros2/common_interfaces/blob/humble/geometry_msgs/msg/Wrench.msg)                           |
| [`proto_aegis_grpc.v1.sensor_msgs_msgs.JointState`](./proto/v1/sensor_msgs.proto)       | `msg`     | JointState         | [`sensor_msgs/msg/JointState`](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/JointState.msg)                       |
| [`proto_aegis_grpc.v1.moveit_facade.GotoJointsRequest`](./proto/v1/control_msgs.proto)  | `msg`     | GotoJointsRequest  | -                                                                                                                                          |
| [`proto_aegis_grpc.v1.moveit_facade.GotoJointsResponse`](./proto/v1/control_msgs.proto) | `msg`     | GotoJointsResponse | -                                                                                                                                          |
| [`proto_aegis_grpc.v1.moveit_facade.GotoPoseRequest`](./proto/v1/control_msgs.proto)    | `msg`     | GotoPoseRequest    | -                                                                                                                                          |
| [`proto_aegis_grpc.v1.moveit_facade.GotoPoseResponse`](./proto/v1/control_msgs.proto)   | `msg`     | GotoPoseResponse   | -                                                                                                                                          |
| [`proto_aegis_grpc.v1.moveit_facade.MoveitService`](./proto/v1/control_msgs.proto)      | `service` | MoveitService      | -                                                                                                                                          |

---

## Credits
* Heavily inspired by [an example](https://github.com/tasada038/ros2_grpc_server) by [Takumi Asada @tasada038](https://github.com/tasada038).
  * Modified version with polished rosdep management can be accessed [here](https://github.com/macmacal/ros2_grpc_server).
