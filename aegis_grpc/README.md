# aegis_grpc

This packages provides a bridge between ROS 2 and [gRPC](https://grpc.io) protocols, with custom [Protobuf](https://protobuf.dev) messages based on selected ones from [`control_msgs`](https://github.com/ros-controls/control_msgs/tree/humble), [`geometry_msgs`](https://github.com/ros2/common_interfaces/tree/humble/geometry_msgs) and [`sensors_mgs`](https://github.com/ros2/common_interfaces/tree/rolling/sensor_msgs) packages.

It aims to provide a form of "frequency clutch" between real-time domain and low-frequnecy decision making (e.g. an ANN inference in Python).

---
# Client (Python)

The client needs to Python packages: 1) [proto_aegis_grpc](./python_proto/README.md) and 2) [aegis_grpc_client](./python_client/README.md).

## Build & install
Use the provided script for automated build & installation of the 2 Python packages:

```bash
bash install_client.sh
```

## Usage
In your code import the Python client and use it freely without ROS dependencies:

```python
import asyncio
from aegis_grpc_client import AegisRobotClient

client = AegisRobotClient("localhost:50051")

async def example_read_sensors():
  """Example: Read robot sensor data."""
  await client.connect()
  try:
      state = await client.get_all()
      print(f"Received robot state: {state}")
  finally:
      await client.disconnect()

asyncio.run(example_read_sensors())

async def example_get_joint_names():
  """Example: Read joint names."""
  await client.connect()
  try:
      names = await client.get_joint_names()
      print(f"Received joint names: {names}")
  finally:
      await client.disconnect()

asyncio.run(example_get_joint_names())
```

---

# Server (ROS 2 C++)

## Usage

### Launch

``` bash
ros2 launch aegis_grpc start_server.launch.py
# or
ros2 run aegis_grpc grpc_server
```

### Interaction
The server has enabled a "reflection" plugin, which allows you to explore all services with external tools like [`grpcurl`](https://github.com/fullstorydev/grpcurl).

```bash
grpcurl -plaintext 127.0.0.1:50051 list
grpcurl -plaintext 127.0.0.1:50051 list proto_aegis_grpc.v1.RobotControlService
grpcurl -plaintext 127.0.0.1:50051 list proto_aegis_grpc.v1.RobotReadService
grpcurl -plaintext 127.0.0.1:50051 describe proto_aegis_grpc.v1.RobotReadService.GetAll
# or with tool from container
podman run --rm --network=host docker.io/fullstorydev/grpcurl -plaintext 127.0.0.1:50051 list
# map grpcurl container alias
alias grpcurl="podman run --rm --network=host docker.io/fullstorydev/grpcurl"
```

Example call to the `GetAll` method with result as a plain json:

```bash
grpcurl -max-msg-sz 10485760 -plaintext -d '{}' 127.0.0.1:50051 proto_aegis_grpc.v1.RobotReadService.GetAll
```

## Messages architecture

Since this project aims to bridge communication between ROS and gRPC, the protobuf definitions tries to mimic default messages, services and actions from ROS.
The main difference is the lack of the header with timestamps - the data synchronization responsibility should not be forwarded outside the ROS ecosystem.

The server is split into 2 services defined in [`proto_aegis_grpc.v1.robot_srvs`](./proto_aegis_grpc/v1/robot_srvs.proto): `RobotReadService` and `RobotControlService`.

### Reading data with `RobotReadService`

The "ROS-getters" are implemented in the [`RobotReadServiceImpl`](./include/aegis_grpc/robot_read_service.hpp) class as the following methods:

| Method name                                                | Desc.                                                | Impl. | gRPC Request            | gRPC Response                                                                               |
| ---------------------------------------------------------- | ---------------------------------------------------- | ----- | ----------------------- | ------------------------------------------------------------------------------------------- |
| `proto_aegis_grpc.v1.RobotReadService.GetAll`              | Get all available robot data.                        | ✅     | `google.protobuf.Empty` | [`proto_aegis_grpc.v1.robot_srvs.RobotObservation`](./proto_aegis_grpc/v1/robot_srvs.proto) |
| `proto_aegis_grpc.v1.RobotReadService.GetRobotState`       | Get consolidated robot state (pose, joints, wrench). | ✅     | `google.protobuf.Empty` | [`proto_aegis_grpc.v1.robot_srvs.RobotState`](./proto_aegis_grpc/v1/robot_srvs.proto)       |
| `proto_aegis_grpc.v1.RobotReadService.GetRobotVision`      | Get all camera images (scene, left, right).          | ✅     | `google.protobuf.Empty` | [`proto_aegis_grpc.v1.robot_srvs.RobotVision`](./proto_aegis_grpc/v1/robot_srvs.proto)      |
| `proto_aegis_grpc.v1.RobotReadService.GetJointStates`      | Get the joints' states.                              | ✅     | `google.protobuf.Empty` | [`proto_aegis_grpc.v1.sensor_msgs.JointState`](./proto_aegis_grpc/v1/sensor_msgs.proto)     |
| `proto_aegis_grpc.v1.RobotReadService.GetTCPPose`          | Get the TCP pose.                                    | ✅     | `google.protobuf.Empty` | [`proto_aegis_grpc.v1.geometry_msgs.Pose`](./proto_aegis_grpc/v1/geometry_msgs.proto)       |
| `proto_aegis_grpc.v1.RobotReadService.GetWrench`           | Read force/torque sensor.                            | ✅     | `google.protobuf.Empty` | [`proto_aegis_grpc.v1.geometry_msgs.Wrench`](./proto_aegis_grpc/v1/geometry_msgs.proto)     |
| `proto_aegis_grpc.v1.RobotReadService.GetCameraSceneImage` | Get scene camera image.                              | ✅     | `google.protobuf.Empty` | [`proto_aegis_grpc.v1.sensor_msgs.Image`](./proto_aegis_grpc/v1/sensor_msgs.proto)          |
| `proto_aegis_grpc.v1.RobotReadService.GetCameraRightImage` | Get right tool camera image.                         | ✅     | `google.protobuf.Empty` | [`proto_aegis_grpc.v1.sensor_msgs.Image`](./proto_aegis_grpc/v1/sensor_msgs.proto)          |
| `proto_aegis_grpc.v1.RobotReadService.GetCameraLeftImage`  | Get left tool camera image.                          | ✅     | `google.protobuf.Empty` | [`proto_aegis_grpc.v1.sensor_msgs.Image`](./proto_aegis_grpc/v1/sensor_msgs.proto)          |

You can always list the methods with the `grpcurl` command:
```bash
grpcurl -plaintext 127.0.0.1:50051 list proto_aegis_grpc.v1.RobotReadService
```

### Controlling the robot with `RobotControlService`

The control bridge for MoveIt2 servo and planners are implemented in the [`RobotControlServiceImpl`](./include/aegis_grpc/robot_control_service.hpp) class as the following methods:

| Method name                                                  | Desc.                                                           | Impl. | gRPC Request                                                                                         | gRPC Response                                                                              |
| ------------------------------------------------------------ | --------------------------------------------------------------- | ----- | ---------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------ |
| `proto_aegis_grpc.v1.RobotControlService.GotoJoints`         | Plan & execute the MoveIt2 trajectory to a given joints target. | ✅     | [`proto_aegis_grpc.v1.sensor_msgs.JointState`](./proto_aegis_grpc/v1/sensor_msgs.proto)              | [`proto_aegis_grpc.v1.robot_srvs.TriggerResponse`](./proto_aegis_grpc/v1/robot_srvs.proto) |
| `proto_aegis_grpc.v1.RobotControlService.GotoPose`           | Plan & execute the MoveIt2 trajectory to a given pose target.   | ✅     | [`proto_aegis_grpc.v1.geometry_msgs.Pose`](./proto_aegis_grpc/v1/geometry_msgs.proto)                | [`proto_aegis_grpc.v1.robot_srvs.TriggerResponse`](./proto_aegis_grpc/v1/robot_srvs.proto) |
| `proto_aegis_grpc.v1.RobotControlService.GripperClose`       | Closes the gripper.                                             | ✅     | `google.protobuf.Empty`                                                                              | [`proto_aegis_grpc.v1.robot_srvs.TriggerResponse`](./proto_aegis_grpc/v1/robot_srvs.proto) |
| `proto_aegis_grpc.v1.RobotControlService.GripperOpen`        | Opens the gripper.                                              | ✅     | `google.protobuf.Empty`                                                                              | [`proto_aegis_grpc.v1.robot_srvs.TriggerResponse`](./proto_aegis_grpc/v1/robot_srvs.proto) |
| `proto_aegis_grpc.v1.RobotControlService.ServoEnable`        | Enable servo control.                                           | ✅     | `google.protobuf.Empty`                                                                              | [`proto_aegis_grpc.v1.robot_srvs.TriggerResponse`](./proto_aegis_grpc/v1/robot_srvs.proto) |
| `proto_aegis_grpc.v1.RobotControlService.ServoDisable`       | Disable servo control.                                          | ✅     | `google.protobuf.Empty`                                                                              | [`proto_aegis_grpc.v1.robot_srvs.TriggerResponse`](./proto_aegis_grpc/v1/robot_srvs.proto) |
| `proto_aegis_grpc.v1.RobotControlService.GripperSetPosition` | Set the gripper width.                                          | ✅     | [`proto_aegis_grpc.v1.robot_srvs.GripperSetPositionRequest`](./proto_aegis_grpc/v1/robot_srvs.proto) | [`proto_aegis_grpc.v1.robot_srvs.TriggerResponse`](./proto_aegis_grpc/v1/robot_srvs.proto) |
| `proto_aegis_grpc.v1.RobotControlService.ServoJoint`         | Servoing (w.r.t. joints).                                       | ✅     | [`proto_aegis_grpc.v1.control_msgs.JointJog`](./proto_aegis_grpc/v1/control_msgs.proto)              | `google.protobuf.Empty`                                                                    |
| `proto_aegis_grpc.v1.RobotControlService.ServoTCP`           | Servoing (w.r.t. TCP).                                          | ✅     | [`proto_aegis_grpc.v1.geometry_msgs.Twist`](./proto_aegis_grpc/v1/geometry_msgs.proto)               | `google.protobuf.Empty`                                                                    |

You can always list the methods with the `grpcurl` command:
```bash
grpcurl -plaintext 127.0.0.1:50051 list proto_aegis_grpc.v1.RobotControlService
```

### Controlling the LED strips with `WledService`

The wled control service is implemented in the [`WledServiceImpl`](./include/aegis_grpc/wled_service.hpp) class with following methods:

| Method name                                                  | Desc.                                                           | Impl. | gRPC Request                                                                                         | gRPC Response                                                                                  |
| ------------------------------------------------------------ | --------------------------------------------------------------- | ----- | ---------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------- |
| `aegis.grpc.v1.WledService.ChangeScene`                      | Change color end effect displayed on selected segment.          | ✅    | [`aegis.grpc.v1.wled_service.ChangeSceneRequest`](./proto_aegis_grpc/v1/wled_service.proto)          | [`aegis.grpc.v1.wled_service.GenericStatusResponse`](./proto_aegis_grpc/v1/wled_service.proto) |
| `aegis.grpc.v1.WledService.DefineScene`                      | Create new color and brightness preset.                         | ✅    | [`aegis.grpc.v1.wled_service.DefineSceneRequest`](./proto_aegis_grpc/v1/wled_service.proto)          | [`aegis.grpc.v1.wled_service.GenericStatusResponse`](./proto_aegis_grpc/v1/wled_service.proto) |
| `aegis.grpc.v1.WledService.GetScenes`                        | Get all scenes.                                                 | ✅    | `google.protobuf.Empty`                                                                              | [`aegis.grpc.v1.wled_service.GetScenesResponse`](./proto_aegis_grpc/v1/wled_service.proto)     |
| `aegis.grpc.v1.WledService.GetSections`                      | Get all sections.                                               | ✅    | `google.protobuf.Empty`                                                                              | [`aegis.grpc.v1.wled_service.GetSectionsResponse`](./proto_aegis_grpc/v1/wled_service.proto)   |
| `aegis.grpc.v1.WledService.StreamEffects`                    | Get all effects.                                                | ✅    | `google.protobuf.Empty`                                                                              | [`aegis.grpc.v1.wled_service.WledEffectsResponse`](./proto_aegis_grpc/v1/wled_service.proto)   |

You can always list the methods with the `grpcurl` command:
```bash
grpcurl -plaintext 127.0.0.1:50051 list aegis.grpc.v1.WledService
```

### All messages

| gRPC message                                                                                         | ROS 2 interface                                                                                                                                        |
| ---------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------ |
| [`proto_aegis_grpc.v1.control_msgs.JointJog`](./proto_aegis_grpc/v1/control_msgs.proto)              | [`control_msgs/msg/JointJog`](https://github.com/ros-controls/control_msgs/blob/humble/control_msgs/msg/JointJog.msg)                                  |
| [`proto_aegis_grpc.v1.geometry_msgs.Vector3`](./proto_aegis_grpc/v1/geometry_msgs.proto)             | [`geometry_msgs/msg/Vector3`](https://github.com/ros2/common_interfaces/blob/humble/geometry_msgs/msg/Vector3.msg)                                     |
| [`proto_aegis_grpc.v1.geometry_msgs.Point`](./proto_aegis_grpc/v1/geometry_msgs.proto)               | [`geometry_msgs/msg/Point`](https://github.com/ros2/common_interfaces/blob/humble/geometry_msgs/msg/Point.msg)                                         |
| [`proto_aegis_grpc.v1.geometry_msgs.Quaternion`](./proto_aegis_grpc/v1/geometry_msgs.proto)          | [`geometry_msgs/msg/Quaternion`](https://github.com/ros2/common_interfaces/blob/humble/geometry_msgs/msg/Quaternion.msg)                               |
| [`proto_aegis_grpc.v1.geometry_msgs.Pose`](./proto_aegis_grpc/v1/geometry_msgs.proto)                | [`geometry_msgs/msg/Pose`](https://github.com/ros2/common_interfaces/blob/humble/geometry_msgs/msg/Pose.msg)                                           |
| [`proto_aegis_grpc.v1.geometry_msgs.Wrench`](./proto_aegis_grpc/v1/geometry_msgs.proto)              | [`geometry_msgs/msg/Wrench`](https://github.com/ros2/common_interfaces/blob/humble/geometry_msgs/msg/Wrench.msg)                                       |
| [`proto_aegis_grpc.v1.geometry_msgs.Twist`](./proto_aegis_grpc/v1/geometry_msgs.proto)               | [`geometry_msgs/msg/Twist`](https://github.com/ros2/common_interfaces/blob/humble/geometry_msgs/msg/Twist.msg)                                         |
| [`proto_aegis_grpc.v1.robot_srvs.RobotState`](./proto_aegis_grpc/v1/robot_srvs.proto)                | -                                                                                                                                                      |
| [`proto_aegis_grpc.v1.robot_srvs.TriggerResponse`](./proto_aegis_grpc/v1/robot_srvs.proto)           | [`std_srvs/srv/Trigger`](https://github.com/ros2/common_interfaces/blob/humble/std_srvs/srv/Trigger.srv)                                               |
| [`proto_aegis_grpc.v1.robot_srvs.GripperSetPositionRequest`](./proto_aegis_grpc/v1/robot_srvs.proto) | [`control_msgs/action/GripperCommand`](https://github.com/ros-controls/control_msgs/blob/humble/control_msgs/action/GripperCommand.action)             |
| [`proto_aegis_grpc.v1.wled_service.ChangeScene`](./proto_aegis_grpc/v1/wled_service.proto)           | [`wled_ros_dirver/wled_interfaces/srv/ChangeScene`](https://github.com/AGH-CEAI/wled_ros_driver/tree/humble-devel/wled_interfaces/srv/ChangeScene.srv) |
| [`proto_aegis_grpc.v1.wled_service.DefineScene`](./proto_aegis_grpc/v1/wled_service.proto)           | [`wled_ros_dirver/wled_interfaces/srv/DefineScene`](https://github.com/AGH-CEAI/wled_ros_driver/tree/humble-devel/wled_interfaces/srv/DefineScene.srv) |
| [`proto_aegis_grpc.v1.wled_service.GetScenes`](./proto_aegis_grpc/v1/wled_service.proto)             | [`wled_ros_dirver/wled_interfaces/srv/GetScenes`](https://github.com/AGH-CEAI/wled_ros_driver/tree/humble-devel/wled_interfaces/srv/GetScenes.srv)     |
| [`proto_aegis_grpc.v1.wled_service.GetSections`](./proto_aegis_grpc/v1/wled_service.proto)           | [`wled_ros_dirver/wled_interfaces/srv/GetSections`](https://github.com/AGH-CEAI/wled_ros_driver/tree/humble-devel/wled_interfaces/srv/GetSections.srv) |

## ROS parameters

There is no hardcoded parameters, everything can be modified via ROS:

### RobotControlService

**Initialization parameters**

| Parameter               | Type     | Default                             | Units   | Description                                                                  |
| ----------------------- | -------- | ----------------------------------- | ------- | ---------------------------------------------------------------------------- |
| `servo_link`            | `string` | `"base_link"`                       | -       | Name of the base link for TCP servoing.                                      |
| `topic_servo_joint`     | `string` | `"/servo_node/delta_joint_cmds"`    | -       | Output topic for joint servo commands.                                       |
| `topic_servo_tcp`       | `string` | `"/servo_node/delta_twist_cmds"`    | -       | Output topic for TCP servo commands.                                         |
| `action_gripper`        | `string` | `"/gripper_controller/gripper_cmd"` | -       | `GripperCommand` action name for the gripper controller.                     |
| `action_timeout_s`      | `double` | `3.0`                               | seconds | Waiting timeout for actions.                                                 |
| `servo_in_rate_hz`      | `double` | `10.0`                              | Hz      | Expected input servo command frequency.                                      |
| `servo_out_rate_hz`     | `double` | `250.0`                             | Hz      | Servo command publish loop frequency.                                        |
| `move_group`            | `string` | `"aegis_arm"`                       | -       | Name of the planning group to control.                                       |
| `joint_tolerance`       | `double` | `0.017`                             | radians | Ignore MoveIt joints call if target closer than this absolute value.         |
| `position_tolerance`    | `double` | `0.001`                             | meters  | Ignore MoveIt pose call if position target closer than this absolute value.  |
| `orientation_tolerance` | `double` | `0.017`                             | radians | Ignore MoveIt pose call if orientation target closer than this absolute val. |

**Runtime parameters**

| Parameter           | Type     | Default | Units  | Description              |
| ------------------- | -------- | ------- | ------ | ------------------------ |
| `r_gripper_close_m` | `double` | `0.0`   | meters | Gripper closed position. |
| `r_gripper_open_m`  | `double` | `0.025` | meters | Gripper open position.   |

### RobotReadService

**Initialization parameters**

| Parameter              | Type     | Default                         | Units | Description                                      |
| ---------------------- | -------- | ------------------------------- | ----- | ------------------------------------------------ |
| `tcp_frame`            | `string` | `"robotiq_hande_end"`           | -     | TF2 frame ID of the end-effector.                |
| `base_frame`           | `string` | `"world"`                       | -     | TF2 base frame ID for EE pose lookup.            |
| `topic_wrench`         | `string` | `"/wrench"`                     | -     | Topic providing force/torque (F/T) data.         |
| `topic_joints`         | `string` | `"/joint_states"`               | -     | Topic providing joint state data.                |
| `topic_camera_scene`   | `string` | `"/cam_scene/rgb/image_rect"`   | -     | Camera scene image topic.                        |
| `topic_camera_right`   | `string` | `"/cam_tool_right/image_color"` | -     | Right tool camera image topic.                   |
| `topic_camera_left`    | `string` | `"/cam_tool_left/image_color"`  | -     | Left tool camera image topic.                    |
| `topics_history_depth` | `int`    | `1`                             | -     | The topics messages history (buffer) size.       |
| `target_image_width`   | `int`    | `64`                            | px    | Target output image width in pixels.             |
| `target_image_height`  | `int`    | `64`                            | px    | Target output image height in pixels.            |
| `enable_image_resize`  | `bool`   | `true`                          | -     | Enable resizing images before sending over gRPC. |

>[!TIP]
> The end-effector's TCP pose is obtained from the tf2 transform (using `tcp_frame` and `base_frame`).

### Run and set examples
The initialization params can be set via ROS 2 run or launch:
```bash
ros2 run aegis_grpc grpc_server \
  --ros-args \
  -p servo_link:=tool0 \
  -p topic_servo_tcp:=/my_servo/delta_twist_cmds \
  -p action_timeout_s:=5.0 \
  -p servo_in_rate_hz:=50.0 \
  -p r_gripper_open_m:=0.030
```

The runtime params can be set via ROS 2 CLI:
```bash
ros2 param set /grpc_server servo_in_rate_hz 20.0
ros2 param set /grpc_server r_gripper_close_m 0.005
```

---

## Development notes

### Install

There is a chance, that the `rosdep` couldn't resolve the versions for the Python. In such case, try:
```bash
pip3 install "protobuf<3.21" "grpcio<1.51" "grpcio-tools<1.51"
```
In case of conflict with `onnx` for `rsl-rl-lib`, try to downgrade it to `1.17.0`:
```bash
pip3 install "onnx==1.17.0"
```

### CLI commands

You can simulate data on ROS topics with these commands:
```bash
ros2 topic pub /tcp_pose geometry_msgs/msg/PoseStamped "{header: auto, pose: {position: {x: 1.0, y: 2.0, z: 3.0}, orientation: {x: 4.0, y: 5.0, z: 6.0, w: 7.0}}}" --once
ros2 topic pub /joint_states sensor_msgs/msg/JointState "{header: auto, name: ['joint1','joint2'], position: [0.0, 1.0], velocity: [2.0, 3.0], effort: [4.0, 5.0]}" --once
ros2 topic pub /wrench geometry_msgs/msg/WrenchStamped "{header: auto, wrench: {force: {x: 1.0, y: 2.0, z: 3.0}, torque: {x: 4.0, y: 5.0, z: 6.0}}}" --once
```

Testing methods with `grpcurl`:
```bash
grpcurl -plaintext -d '{}' 127.0.0.1:50051 proto_aegis_grpc.v1.RobotReadService.GetAll
grpcurl -plaintext -d '{}' 127.0.0.1:50051 proto_aegis_grpc.v1.RobotControlService.GripperClose
grpcurl -plaintext -d '{}' 127.0.0.1:50051 proto_aegis_grpc.v1.RobotControlService.GripperOpen
grpcurl -plaintext -d '{"position": "0.01"}' 127.0.0.1:50051 proto_aegis_grpc.v1.RobotControlService.GripperSetPosition
grpcurl -plaintext -d '{"linear": {"x": "1.0", "y": "2", "z": "3"}, "angular": {"x": "5", "y": "6", "z": 7}}' 127.0.0.1:50051 proto_aegis_grpc.v1.RobotControlService.ServoTCP
grpcurl -plaintext -d '{"joint_names": ["joint1", "joint2"], "displacements": ["1"], "velocities": ["2", "3"], "duration": "4"}' 127.0.0.1:50051 proto_aegis_grpc.v1.RobotControlService.ServoJoint
grpcurl -plaintext -d '{"joint_names": ["joint1", "joint2"], "displacements": ["1", "2"], "velocities": ["3"], "duration": "4"}' 127.0.0.1:50051 proto_aegis_grpc.v1.RobotControlService.ServoJoint
grpcurl -plaintext -d '{"joint_names": ["joint1", "joint2"], "displacements": ["1", "2"], "velocities": ["3", "4"], "duration": "5"}' 127.0.0.1:50051 proto_aegis_grpc.v1.RobotControlService.ServoJoint
```

---

## Credits
* Heavily inspired by [an example](https://github.com/tasada038/ros2_grpc_server) by [Takumi Asada @tasada038](https://github.com/tasada038).
  * Modified version with polished rosdep management can be accessed [here](https://github.com/macmacal/ros2_grpc_server).
