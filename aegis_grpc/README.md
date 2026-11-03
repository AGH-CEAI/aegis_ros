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
| `proto_aegis_grpc.v1.RobotControlService.WrenchBiasSet`      | Set bias for net FT measurements (aka sensor taring/zeroing).   | ✅     | `google.protobuf.Empty`                                                                              | [`proto_aegis_grpc.v1.robot_srvs.TriggerResponse`](./proto_aegis_grpc/v1/robot_srvs.proto) |
| `proto_aegis_grpc.v1.RobotControlService.WrenchBiasClear`    | Remove net FT measurements bias.                                | ✅     | `google.protobuf.Empty`                                                                              | [`proto_aegis_grpc.v1.robot_srvs.TriggerResponse`](./proto_aegis_grpc/v1/robot_srvs.proto) |

You can always list the methods with the `grpcurl` command:
```bash
grpcurl -plaintext 127.0.0.1:50051 list proto_aegis_grpc.v1.RobotControlService
```

### Compliant TCP control with `AdmittanceService`

The [`AdmittanceServiceImpl`](./include/aegis_grpc/admittance_service.hpp) drives the ros2_control admittance chain
`admittance_jtc` -> `admittance_controller` -> UR (see `aegis_control/config/controllers/admittance.yaml`),
defined in [`proto_aegis_grpc.v1.admittance_service`](./proto_aegis_grpc/v1/admittance_service.proto):

| Method name                                           | Desc.                                                                               | Impl. | gRPC Request                                                                                      | gRPC Response                                                                                                 |
| ----------------------------------------------------- | ----------------------------------------------------------------------------------- | ----- | ------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------- |
| `proto_aegis_grpc.v1.AdmittanceService.Enable`        | Switch the arm to the admittance chain (refused while Servo is enabled).            | ✅     | `google.protobuf.Empty`                                                                           | [`proto_aegis_grpc.v1.robot_srvs.TriggerResponse`](./proto_aegis_grpc/v1/robot_srvs.proto)                    |
| `proto_aegis_grpc.v1.AdmittanceService.Disable`       | Relax the reference, switch back to the MoveIt joint controller.                    | ✅     | `google.protobuf.Empty`                                                                           | [`proto_aegis_grpc.v1.robot_srvs.TriggerResponse`](./proto_aegis_grpc/v1/robot_srvs.proto)                    |
| `proto_aegis_grpc.v1.AdmittanceService.StepTCP`       | Move the TCP reference by a displacement (translation + rotation vector, `world`). | ✅     | [`proto_aegis_grpc.v1.admittance_service.PoseDelta`](./proto_aegis_grpc/v1/admittance_service.proto) | [`proto_aegis_grpc.v1.admittance_service.AdmittanceStepResponse`](./proto_aegis_grpc/v1/admittance_service.proto) |
| `proto_aegis_grpc.v1.AdmittanceService.SetTCPTarget`  | Move the TCP reference towards an absolute pose (same per-call limits).            | ✅     | [`proto_aegis_grpc.v1.geometry_msgs.Pose`](./proto_aegis_grpc/v1/geometry_msgs.proto)             | [`proto_aegis_grpc.v1.admittance_service.AdmittanceStepResponse`](./proto_aegis_grpc/v1/admittance_service.proto) |
| `proto_aegis_grpc.v1.AdmittanceService.GetWrench`     | Wrench used by the admittance controller (filtered, gravity compensated, base frame). | ✅     | `google.protobuf.Empty`                                                                           | [`proto_aegis_grpc.v1.geometry_msgs.Wrench`](./proto_aegis_grpc/v1/geometry_msgs.proto)                       |

How the TCP reference is generated (each `StepTCP` / `SetTCPTarget` call):
1. The displacement is rate limited (`admittance_max_step_lin`, `admittance_max_step_rot`).
2. It is integrated from the previous **reference**, not from the measured pose, so the admittance spring is not
   absorbed by the next command (MoveIt Servo re-bases on the measured state and cannot be used upstream of the admittance).
3. The position is clamped to the optional workspace box.
4. The reference is kept within `admittance_max_offset_*` from the measured pose. This bounds the force the robot applies
   to roughly `stiffness * max_offset` (10 N with the default 500 N/m and 20 mm); a larger external force pushes the robot away.
5. Damped least squares IK warm-started from the previous solution (no IK branch flips), joint-step limit,
   and a MoveIt collision check (`/check_state_validity`, loaded by default by `move_group`).
6. The joint target is sent to `admittance_jtc` as a single-point trajectory lasting `admittance_step_duration_s`
   (set it to the policy period); the JTC interpolates it at the controller rate.

A rejected call (IK, joint step, collision) returns `success=false` and keeps the previous reference.
If the chain was stopped externally (e.g. by `controller_stopper` after a protective stop), the calls fail with
`FAILED_PRECONDITION` until `Enable` is called again.

Typical episode:
```python
await client.admittance_disable()  # no-op if not enabled
await client.goto_pose(start_pos, start_quat)  # reset with MoveIt (scaled JTC)
await (
    client.wrench_bias_set()
)  # tare the F/T sensor in the start pose (see admittance.yaml)
await client.admittance_enable()
for _ in range(steps):  # e.g. 10 Hz
    obs_wrench = await client.admittance_get_wrench()
    ok, msg, clipped, ref = await client.admittance_step(dp, drot)
await client.admittance_disable()
```

You can always list the methods with the `grpcurl` command:
```bash
grpcurl -plaintext 127.0.0.1:50051 list proto_aegis_grpc.v1.AdmittanceService
```

### Controlling the LED strips with `WledService`

The wled control service is implemented in the [`WledServiceImpl`](./include/aegis_grpc/wled_service.hpp) class with following methods:

| Method name                                   | Desc.                                                  | Impl. | gRPC Request                                                                                      | gRPC Response                                                                                        |
| --------------------------------------------- | ------------------------------------------------------ | ----- | ------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------- |
| `proto_aegis_grpc.v1.WledService.ChangeScene` | Change color end effect displayed on selected segment. | ✅     | [`proto_aegis_grpc.v1.wled_service.ChangeSceneRequest`](./proto_aegis_grpc/v1/wled_service.proto) | [`proto_aegis_grpc.v1.wled_service.GenericStatusResponse`](./proto_aegis_grpc/v1/wled_service.proto) |
| `proto_aegis_grpc.v1.WledService.DefineScene` | Create new color and brightness preset.                | ✅     | [`proto_aegis_grpc.v1.wled_service.DefineSceneRequest`](./proto_aegis_grpc/v1/wled_service.proto) | [`proto_aegis_grpc.v1.wled_service.GenericStatusResponse`](./proto_aegis_grpc/v1/wled_service.proto) |
| `proto_aegis_grpc.v1.WledService.GetScenes`   | Get all scenes.                                        | ✅     | `google.protobuf.Empty`                                                                           | [`proto_aegis_grpc.v1.wled_service.GetScenesResponse`](./proto_aegis_grpc/v1/wled_service.proto)     |
| `proto_aegis_grpc.v1.WledService.GetSections` | Get all sections.                                      | ✅     | `google.protobuf.Empty`                                                                           | [`proto_aegis_grpc.v1.wled_service.GetSectionsResponse`](./proto_aegis_grpc/v1/wled_service.proto)   |
| `proto_aegis_grpc.v1.WledService.GetEffects`  | Get all effects.                                       | ✅     | `google.protobuf.Empty`                                                                           | [`proto_aegis_grpc.v1.wled_service.WledEffectsResponse`](./proto_aegis_grpc/v1/wled_service.proto)   |

You can always list the methods with the `grpcurl` command:
```bash
grpcurl -plaintext 127.0.0.1:50051 list proto_aegis_grpc.v1.WledService
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

| Parameter                   | Type     | Default                             | Units   | Description                                                                  |
| --------------------------- | -------- | ----------------------------------- | ------- | ---------------------------------------------------------------------------- |
| `servo_link`                | `string` | `"base_link"`                       | -       | Name of the base link for TCP servoing.                                      |
| `topic_servo_joint`         | `string` | `"/servo_node/delta_joint_cmds"`    | -       | Output topic for joint servo commands.                                       |
| `topic_servo_tcp`           | `string` | `"/servo_node/delta_twist_cmds"`    | -       | Output topic for TCP servo commands.                                         |
| `action_gripper`            | `string` | `"/gripper_controller/gripper_cmd"` | -       | `GripperCommand` action name for the gripper controller.                     |
| `action_timeout_s`          | `double` | `3.0`                               | seconds | Waiting timeout for actions.                                                 |
| `servo_in_rate_hz`          | `double` | `10.0`                              | Hz      | Expected input servo command frequency.                                      |
| `servo_out_rate_hz`         | `double` | `250.0`                             | Hz      | Servo command publish loop frequency.                                        |
| `move_group`                | `string` | `"aegis_arm"`                       | -       | Name of the planning group to control.                                       |
| `joint_tolerance`           | `double` | `0.017`                             | radians | Ignore MoveIt joints call if target closer than this absolute value.         |
| `position_tolerance`        | `double` | `0.001`                             | meters  | Ignore MoveIt pose call if position target closer than this absolute value.  |
| `orientation_tolerance`     | `double` | `0.017`                             | radians | Ignore MoveIt pose call if orientation target closer than this absolute val. |
| `service_wrench_bias_set`   | `string` | `"/net_ft_bias/set_bias"`           | -       | Service name for setting the net FT sensor bias (zeroing/taring).            |
| `service_wrench_bias_clear` | `string` | `"/net_ft_bias/clear_bias"`         | -       | Service name for clearing the net FT sensor bias.                            |

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
> The end-effector's TCP pose can be obtained from the tf2 transform (using `tcp_frame` and `base_frame`).

### WledService

**Initialization parameters**

| Parameter              | Type     | Default                         | Units | Description                                        |
| ---------------------- | -------- | ------------------------------- | ----- | -------------------------------------------------- |
| `wled_effects_topic`   | `string` | `"/wled_effects"`               | -     | Topic providing serialized dict with WLED effects. |

### AdmittanceService

**Initialization parameters**

| Parameter                              | Type       | Default                                                  | Units   | Description                                                                                   |
| -------------------------------------- | ---------- | -------------------------------------------------------- | ------- | --------------------------------------------------------------------------------------------- |
| `admittance_group`                     | `string`   | `"aegis_arm"`                                            | -       | MoveIt planning group controlled by the chain.                                                |
| `admittance_tcp_link`                  | `string`   | `"robotiq_hande_end"`                                    | -       | Commanded link (TCP).                                                                         |
| `admittance_controllers`               | `string[]` | `["admittance_controller", "admittance_jtc"]`            | -       | Controllers of the chain, activated together.                                                 |
| `admittance_servo_controllers`         | `string[]` | `["forward_position_controller", "forward_velocity_controller"]` | - | `Enable` is refused while one of them is active (Servo mode).                                 |
| `admittance_fallback_controller`       | `string`   | `"scaled_joint_trajectory_controller"`                   | -       | Controller activated by `Disable` (used by MoveIt). The launch file sets the mock one.         |
| `admittance_topic_trajectory`          | `string`   | `"/admittance_jtc/joint_trajectory"`                     | -       | Pub: trajectory topic of the chained JTC.                                                     |
| `admittance_topic_status`              | `string`   | `"/admittance_controller/status"`                        | -       | Sub: admittance controller state (wrench, liveness).                                          |
| `admittance_topic_joints`              | `string`   | `"/joint_states"`                                        | -       | Sub: measured joint states.                                                                   |
| `admittance_step_duration_s`           | `double`   | `0.1`                                                    | seconds | Duration of each reference segment; set to the policy period.                                 |
| `admittance_settle_time_s`             | `double`   | `0.3`                                                    | seconds | Relaxation time before `Disable` switches controllers.                                        |
| `admittance_max_step_lin`              | `double`   | `0.01`                                                   | meters  | Max TCP translation per call.                                                                 |
| `admittance_max_step_rot`              | `double`   | `0.05`                                                   | radians | Max TCP rotation per call.                                                                    |
| `admittance_max_offset_lin`            | `double`   | `0.02`                                                   | meters  | Max reference-to-measured translation (force bound). Keep >= 2x `max_step_lin`.               |
| `admittance_max_offset_rot`            | `double`   | `0.1`                                                    | radians | Max reference-to-measured rotation. Keep >= 2x `max_step_rot`.                                |
| `admittance_workspace_min`/`_max`      | `double[3]`| `[]` (disabled)                                          | meters  | Workspace box in the root frame.                                                              |
| `admittance_max_joint_step`            | `double`   | `0.2`                                                    | radians | Reject IK solutions moving any joint more than this per call.                                 |
| `admittance_ik_damping`                | `double`   | `0.01`                                                   | -       | Damped least squares factor.                                                                  |
| `admittance_ik_max_iterations`         | `int`      | `20`                                                     | -       | IK iterations per call.                                                                       |
| `admittance_check_collisions`          | `bool`     | `true`                                                   | -       | Validate each reference with MoveIt `/check_state_validity`.                                   |
| `admittance_collision_check_timeout_s` | `double`   | `0.1`                                                    | seconds | Timeout of a single collision check.                                                          |

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
