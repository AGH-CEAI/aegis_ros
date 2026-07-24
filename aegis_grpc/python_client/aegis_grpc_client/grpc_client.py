import logging
from enum import Enum, auto
from typing import Optional, Union
from contextlib import asynccontextmanager

import numpy as np
import grpc
from google.protobuf.empty_pb2 import Empty
from strenum import StrEnum, LowercaseStrEnum

from proto_aegis_grpc.v1 import (
    robot_srvs_pb2,
    robot_srvs_pb2_grpc,
    geometry_msgs_pb2,
    control_msgs_pb2,
    sensor_msgs_pb2,
)


class ModalityGroup(StrEnum):
    STATE = "state"
    VISION = "vision"


class CameraName(StrEnum):
    CAMERA_SCENE = "scene"
    CAMERA_TOOL_LEFT = "tool_left"
    CAMERA_TOOL_RIGHT = "tool_right"


class StateModality(StrEnum):
    POSE = "pose"
    JOINTS = "joints"
    WRENCH = "wrench"


class AegisJointName(LowercaseStrEnum):
    ROBOTIQ_HANDE_LEFT_FINGER_JOINT = auto()
    SHOULDER_PAN_JOINT = auto()
    WRIST_3_JOINT = auto()
    WRIST_2_JOINT = auto()
    WRIST_1_JOINT = auto()
    ELBOW_JOINT = auto()
    SHOULDER_LIFT_JOINT = auto()


AEGIS_JOINTS_ORDER: tuple[AegisJointName, ...] = (
    AegisJointName.SHOULDER_PAN_JOINT,
    AegisJointName.SHOULDER_LIFT_JOINT,
    AegisJointName.ELBOW_JOINT,
    AegisJointName.WRIST_1_JOINT,
    AegisJointName.WRIST_2_JOINT,
    AegisJointName.WRIST_3_JOINT,
    AegisJointName.ROBOTIQ_HANDE_LEFT_FINGER_JOINT,
)


class AegisJointIndex(Enum):
    SHOULDER_PAN_JOINT = 0
    SHOULDER_LIFT_JOINT = 1
    ELBOW_JOINT = 2
    WRIST_1_JOINT = 3
    WRIST_2_JOINT = 4
    WRIST_3_JOINT = 5
    ROBOTIQ_HANDE_LEFT_FINGER_JOINT = 6


class PrefixedLoggerAdapter(logging.LoggerAdapter):
    def process(self, msg, kwargs):
        return f"[aegis_robot_client] {msg}", kwargs


class AegisRobotClient:
    """
    Async gRPC client for Aegis Robot Control System.
    Provides convenient wrappers for:
    - RobotReadService: Getting sensor data (TCP pose, wrench, joint states)
    - RobotControlService: Commanding robot motion (servo, goto, gripper)
    """

    def __init__(self, server_address: str = "127.0.0.1:50051"):
        """
        Initialize the client.

        Args:
            server_address: gRPC server address in format "host:port"
        """
        self.logger = PrefixedLoggerAdapter(logging.getLogger("aegis_grpc_client"), {})
        self.server_address = server_address
        self.channel: Optional[grpc.aio.Channel] = None
        self.read_stub: Optional[robot_srvs_pb2_grpc.RobotReadServiceStub] = None
        self.control_stub: Optional[robot_srvs_pb2_grpc.RobotControlServiceStub] = None
        self._connected = False

    @property
    def is_connected(self) -> bool:
        return self._connected

    async def connect(self) -> None:
        """Establish connection to the gRPC server."""
        try:
            self.channel = grpc.aio.insecure_channel(self.server_address)
            self.read_stub = robot_srvs_pb2_grpc.RobotReadServiceStub(self.channel)
            self.control_stub = robot_srvs_pb2_grpc.RobotControlServiceStub(
                self.channel
            )
            self._connected = True
            self.logger.info(f"Connected to gRPC server at {self.server_address}")
        except Exception as e:
            self.logger.error(f"Failed to connect: {e}")
            raise

    async def disconnect(self) -> None:
        """Close the gRPC connection."""
        if self.channel:
            await self.channel.close()
            self._connected = False
            self.logger.info("Disconnected from gRPC server")

    @asynccontextmanager
    async def connect_context(self):
        """Context manager for automatic connect/disconnect."""
        await self.connect()
        try:
            yield self
        finally:
            await self.disconnect()

    def _check_connected(self) -> None:
        """Verify client is connected."""
        if not self._connected:
            raise RuntimeError(
                "Client not connected. Call `await AegisRobotClient.connect()` first."
            )

    # ==================== RobotReadService Methods ====================

    async def get_tcp_pose(self) -> np.ndarray:
        """
        Get current TCP (Tool Center Point) pose.

        Returns:
            Numpy array: 7D [x, y, z, qx, qy, qz, qw]
        """
        self._check_connected()
        try:
            pose = await self.read_stub.GetTCPPose(Empty())
            return self._pose_to_array(pose)
        except grpc.RpcError as e:
            self.logger.error(f"GetTCPPose failed: {e}")
            raise

    async def get_wrench(self) -> np.ndarray:
        """
        Get current force and torque measurements.

        Returns:
            Numpy array: 6D [fx,fy,fz,tx,ty,tz]
        """
        self._check_connected()
        try:
            wrench = await self.read_stub.GetWrench(Empty())
            return self._wrench_to_array(wrench)
        except grpc.RpcError as e:
            self.logger.error(f"GetWrench failed: {e}")
            raise

    async def get_joint_states(self) -> np.ndarray:
        """
        Get current joint positions, velocities, and efforts.

        Returns:
            Numpy array: [n_joints, 3] of [pos, vel, eff].
        """
        self._check_connected()
        try:
            joint_state = await self.read_stub.GetJointStates(Empty())
            return self._joints_to_array(joint_state)
        except grpc.RpcError as e:
            self.logger.error(f"GetJointStates failed: {e}")
            raise

    async def get_joint_names(self) -> tuple[str]:
        """
        Get joints names.

        Returns:
            tuple of strings
        """
        self._check_connected()
        try:
            joint_state = await self.read_stub.GetJointStates(Empty())
            return tuple(joint_state.name)
        except grpc.RpcError as e:
            self.logger.error(f"GetJointStates (names) failed: {e}")
            raise

    async def get_robot_state(self) -> dict[str, np.ndarray]:
        self._check_connected()
        try:
            state = await self.read_stub.GetRobotState(Empty())
            return {
                StateModality.POSE: self._pose_to_array(state.pose),
                StateModality.WRENCH: self._wrench_to_array(state.wrench),
                StateModality.JOINTS: self._joints_to_array(state.joint_state),
            }
        except grpc.RpcError as e:
            self.logger.error(f"GetRobotState failed: {e}")
            raise

    async def get_camera_scene_image(self) -> np.ndarray:
        self._check_connected()
        try:
            vision = await self.read_stub.GetCameraSceneImage(Empty())
            return self._image_to_array(vision)
        except grpc.RpcError as e:
            self.logger.error(f"GetCameraSceneImage failed: {e}")
            raise

    async def get_camera_right_image(self) -> np.ndarray:
        self._check_connected()
        try:
            vision = await self.read_stub.GetCameraRightImage(Empty())
            return self._image_to_array(vision)
        except grpc.RpcError as e:
            self.logger.error(f"GetCameraRightImage failed: {e}")
            raise

    async def get_camera_left_image(self) -> np.ndarray:
        self._check_connected()
        try:
            vision = await self.read_stub.GetCameraLeftImage(Empty())
            return self._image_to_array(vision)
        except grpc.RpcError as e:
            self.logger.error(f"GetCameraLeftImage failed: {e}")
            raise

    async def get_robot_vision(self) -> dict[str, np.ndarray]:
        self._check_connected()
        try:
            vision = await self.read_stub.GetRobotVision(Empty())
            return {
                CameraName.CAMERA_SCENE: self._image_to_array(vision.image_scene),
                CameraName.CAMERA_TOOL_RIGHT: self._image_to_array(vision.image_right),
                CameraName.CAMERA_TOOL_LEFT: self._image_to_array(vision.image_left),
            }
        except grpc.RpcError as e:
            self.logger.error(f"GetRobotVision failed: {e}")
            raise

    async def get_all(self) -> dict[str, object]:
        self._check_connected()
        try:
            obs = await self.read_stub.GetAll(Empty())
            return {
                ModalityGroup.STATE: {
                    StateModality.POSE: self._pose_to_array(obs.robot_state.pose),
                    StateModality.WRENCH: self._wrench_to_array(obs.robot_state.wrench),
                    StateModality.JOINTS: self._joints_to_array(
                        obs.robot_state.joint_state
                    ),
                },
                ModalityGroup.VISION: {
                    CameraName.CAMERA_SCENE: self._image_to_array(
                        obs.robot_vision.image_scene
                    ),
                    CameraName.CAMERA_TOOL_RIGHT: self._image_to_array(
                        obs.robot_vision.image_right
                    ),
                    CameraName.CAMERA_TOOL_LEFT: self._image_to_array(
                        obs.robot_vision.image_left
                    ),
                },
            }
        except grpc.RpcError as e:
            self.logger.error(f"GetAll failed: {e}")
            raise

    def _pose_to_array(self, pose: geometry_msgs_pb2.Pose) -> np.ndarray:
        return np.array(
            [
                pose.position.x,
                pose.position.y,
                pose.position.z,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ],
            dtype=np.float32,
        )

    def _wrench_to_array(self, wrench: geometry_msgs_pb2.Wrench) -> np.ndarray:
        return np.array(
            [
                wrench.force.x,
                wrench.force.y,
                wrench.force.z,
                wrench.torque.x,
                wrench.torque.y,
                wrench.torque.z,
            ],
            dtype=np.float32,
        )

    def _joints_to_array(self, joints: sensor_msgs_pb2.JointState) -> np.ndarray:
        return np.array(
            [joints.position, joints.velocity, joints.effort], dtype=np.float32
        ).T

    def _image_to_array(self, img: sensor_msgs_pb2.Image) -> np.ndarray:
        return np.moveaxis(
            np.frombuffer(img.data, dtype=np.uint8).reshape(img.height, img.width, 3),
            -1,
            0,
        )

    # ==================== RobotControlService Methods ====================

    async def servo_joint(
        self,
        joint_names: list[str],
        displacements: Optional[Union[list[float], np.ndarray]] = None,
        velocities: Optional[Union[list[float], np.ndarray]] = None,
        duration: float = 0.01,
    ) -> None:
        """
        Perform joint servo (jog individual joints).

        Args:
            joint_names: List of joint names to command
            displacements: Position commands in meters or radians (optional)
            velocities: Velocity commands in m/s or rad/s (optional)
            duration: Duration of command in seconds
        """
        self._check_connected()

        if isinstance(joint_names, np.ndarray):
            joint_names = joint_names.tolist()
        if isinstance(displacements, np.ndarray):
            displacements = displacements.tolist()
        if isinstance(velocities, np.ndarray):
            velocities = velocities.tolist()

        jog = control_msgs_pb2.JointJog(
            joint_names=joint_names,
            displacements=displacements or [],
            velocities=velocities or [],
            duration=duration,
        )

        try:
            await self.control_stub.ServoJoint(jog)
        except grpc.RpcError as e:
            self.logger.error(f"ServoJoint failed: {e}")
            raise

    async def servo_tcp(
        self,
        linear: Union[tuple[float, float, float], np.ndarray],
        angular: Union[tuple[float, float, float], np.ndarray],
    ) -> None:
        """
        Perform TCP servo with Cartesian velocity control.

        Args:
            linear: Linear velocity (vx, vy, vz) in m/s
            angular: Angular velocity (wx, wy, wz) in rad/s
        """
        self._check_connected()

        if isinstance(linear, np.ndarray):
            assert linear.shape == (3,), (
                f"Expected linear shape (3,), got {linear.shape}"
            )
            linear = tuple(linear)
        if isinstance(angular, np.ndarray):
            assert angular.shape == (3,), (
                f"Expected angular shape (3,), got {angular.shape}"
            )
            angular = tuple(angular)

        try:
            twist = geometry_msgs_pb2.Twist(
                linear=geometry_msgs_pb2.Vector3(x=linear[0], y=linear[1], z=linear[2]),
                angular=geometry_msgs_pb2.Vector3(
                    x=angular[0], y=angular[1], z=angular[2]
                ),
            )
        except (IndexError, ValueError) as e:
            self.logger.error(
                f"Failed to convert np.array into Twist protobuf message: {e}"
            )
            raise

        try:
            await self.control_stub.ServoTCP(twist)
        except grpc.RpcError as e:
            self.logger.error(f"ServoTCP failed: {e}")
            raise

    async def goto_pose(
        self,
        position: Union[tuple[float, float, float], np.ndarray],
        orientation: Union[tuple[float, float, float, float], np.ndarray],
    ) -> tuple[bool, str]:
        """
        Move robot to specified TCP pose.

        Args:
            pose: Target Pose with position and orientation (quaternion)

        Returns:
            (success, message) tuple
        """
        self._check_connected()

        if isinstance(position, np.ndarray):
            position = tuple(position)
        if isinstance(orientation, np.ndarray):
            orientation = tuple(orientation)

        pose = geometry_msgs_pb2.Pose(
            position=geometry_msgs_pb2.Point(
                x=position[0],
                y=position[1],
                z=position[2],
            ),
            orientation=geometry_msgs_pb2.Quaternion(
                x=orientation[0],
                y=orientation[1],
                z=orientation[2],
                w=orientation[3],
            ),
        )

        try:
            response = await self.control_stub.GotoPose(pose)
            return response.success, response.msg
        except grpc.RpcError as e:
            self.logger.error(f"Failed to send robot motion command: {e}")
            raise

    async def goto_joints(
        self,
        names: tuple[str],
        positions: Union[tuple[float], np.ndarray],
    ) -> tuple[bool, str]:
        """
        Move robot to specified joint configuration.

        Args:
            joint_state: Target JointState with names and positions

        Returns:
            (success, message) tuple
        """
        self._check_connected()

        if isinstance(positions, np.ndarray):
            positions = tuple(positions)

        joint_state = sensor_msgs_pb2.JointState(
            name=names,
            position=positions,
        )

        try:
            response = await self.control_stub.GotoJoints(joint_state)
            return response.success, response.msg
        except grpc.RpcError as e:
            self.logger.error(f"Failed to send robot motion command: {e}")
            raise

    async def gripper_set_position(
        self, position: float, effort: float
    ) -> tuple[bool, str]:
        """
        Set gripper to specified position and effort.

        Args:
            position: Target gripper gap size in meters
            effort: Goal effort in Newtons

        Returns:
            (success, message) tuple
        """
        self._check_connected()

        request = robot_srvs_pb2.GripperSetPositionRequest(
            position=position, effort=effort
        )

        try:
            response = await self.control_stub.GripperSetPosition(request)
            return response.success, response.msg
        except grpc.RpcError as e:
            self.logger.error(f"GripperSetPosition failed: {e}")
            raise

    async def gripper_close(self) -> tuple[bool, str]:
        """Close the gripper."""
        self._check_connected()
        try:
            response = await self.control_stub.GripperClose(Empty())
            return response.success, response.msg
        except grpc.RpcError as e:
            self.logger.error(f"GripperClose failed: {e}")
            raise

    async def gripper_open(self) -> tuple[bool, str]:
        """Open the gripper."""
        self._check_connected()
        try:
            response = await self.control_stub.GripperOpen(Empty())
            return response.success, response.msg
        except grpc.RpcError as e:
            self.logger.error(f"GripperOpen failed: {e}")
            raise

    async def servo_enable(self) -> tuple[bool, str]:
        self._check_connected()
        try:
            resp = await self.control_stub.ServoEnable(Empty())
            return resp.success, resp.msg
        except grpc.RpcError as e:
            raise RuntimeError("Failed to enable servo") from e

    async def servo_disable(self) -> tuple[bool, str]:
        self._check_connected()
        try:
            resp = await self.control_stub.ServoDisable(Empty())
            return resp.success, resp.msg
        except grpc.RpcError as e:
            raise RuntimeError("Failed to disable servo") from e
