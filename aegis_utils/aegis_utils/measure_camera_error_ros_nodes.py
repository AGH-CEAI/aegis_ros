import threading
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from dataclasses import dataclass
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import Buffer, TransformListener, TransformBroadcaster

from aegis_director import RobotDirector
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from ur_dashboard_msgs.srv import IsInRemoteControl, GetProgramState
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger


@dataclass
class TcpPosCamera:
    retval: bool
    rvec: np.ndarray
    tvec: np.ndarray


class CalibrationTool(Node):
    def __init__(self, tool_offset: list[float]) -> None:
        super().__init__("static_tf_broadcaster")
        self.broadcaster = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "tool0"
        t.child_frame_id = "calibration_tool"

        t.transform.translation.x = tool_offset[0]
        t.transform.translation.y = tool_offset[1]
        t.transform.translation.z = tool_offset[2]
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.broadcaster.sendTransform(t)
        self.get_logger().info("Static transform tool0 -> calibration_tool published")


class CollectImageNode(Node):
    def __init__(self, robot: RobotDirector, image_topic: str, res_path: Path) -> None:
        super().__init__("collect_image_node")
        self.robot = robot
        self.image = None
        self.timestamp = None
        self.bridge = CvBridge()
        self.res_path = res_path
        self.mutex = threading.Lock()
        self.object_tf_brodcaster = TransformBroadcaster(self)
        self.sub = self.create_subscription(Image, image_topic, self.image_callback, 10)
        self.get_logger().info(f"Subscribed to image topic: {image_topic}")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.last_object_tf = None
        self.last_tf_time = None
        self.create_timer(0.1, self._republish_last_tf)

    def image_callback(self, msg: Image) -> None:
        try:
            with self.mutex:
                self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                self.timestamp = self.get_timestamp(msg.header.stamp)
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")

    def get_timestamp(self, time_msg: Time) -> float:
        return time_msg.sec + time_msg.nanosec * 1e-9

    def _get_time_elapsed(self, start_time: float) -> float:
        return self.get_clock().now().nanoseconds / 1e9 - start_time

    def _should_return_image(self, time_start: float) -> bool:
        has_valid_data = self.image is not None and self.timestamp is not None
        is_fresh_data = (self.timestamp > time_start) if has_valid_data else False
        return has_valid_data and is_fresh_data

    def get_image(self, time_start: float, timeout: float = 3.0) -> np.ndarray | None:
        while self._get_time_elapsed(time_start) < timeout:
            with self.mutex:
                if self._should_return_image(time_start):
                    return self.image.copy()
            time.sleep(1)
        return None

    def save_image(self, image: np.ndarray | None, path: Path) -> None:
        if image is None:
            self.get_logger().warn("No image to save")
            return
        cv2.imwrite(str(path), image)
        self.get_logger().info(f"Saved image to: {path}")

    def get_calibration_tool_pose_in_base(self) -> dict[str, np.ndarray]:
        """
        Returns (position, orientation) of calibration_tool in base frame.

        position: np.array([x, y, z])
        orientation: np.array([qx, qy, qz, qw])
        """
        try:
            t = self.tf_buffer.lookup_transform(
                "base_link",  # target frame
                "calibration_tool",  # source frame
                Time(),  # latest available
            )
        except Exception as e:
            self.get_logger().error(
                f"Failed to lookup transform base->calibration_tool: {e}"
            )
            return None

        pos = np.array(
            [
                t.transform.translation.x,
                t.transform.translation.y,
                t.transform.translation.z,
            ],
            dtype=float,
        )

        ori = np.array(
            [
                t.transform.rotation.x,
                t.transform.rotation.y,
                t.transform.rotation.z,
                t.transform.rotation.w,
            ],
            dtype=float,
        )

        return {"position": pos, "orientation": ori}

    def get_object_pose_in_base(self) -> dict[str, np.ndarray]:
        """
        Returns (position, orientation) of calibration_tool in base frame.

        position: np.array([x, y, z])
        orientation: np.array([qx, qy, qz, qw])
        """
        try:
            t = self.tf_buffer.lookup_transform(
                "base_link",  # target frame
                "detected_object",  # source frame
                Time(),  # latest available
            )
        except Exception as e:
            self.get_logger().error(
                f"Failed to lookup transform base->detected_object: {e}"
            )
            return None

        pos = np.array(
            [
                t.transform.translation.x,
                t.transform.translation.y,
                t.transform.translation.z,
            ],
            dtype=float,
        )

        ori = np.array(
            [
                t.transform.rotation.x,
                t.transform.rotation.y,
                t.transform.rotation.z,
                t.transform.rotation.w,
            ],
            dtype=float,
        )

        return {"position": pos, "orientation": ori}

    def _republish_last_tf(self):
        if self.last_object_tf is None:
            return
        rvec, tvec = self.last_object_tf
        tf_camera_name = "cam_scene_rgb_camera_optical_frame"
        self._publish_detected_object_tf(
            rvec=rvec,
            tvec=tvec,
            parent_frame=tf_camera_name,
            child_frame="detected_object",
        )

    def _publish_detected_object_tf(
        self,
        rvec: np.ndarray,
        tvec: np.ndarray,
        parent_frame: str,
        child_frame: str,
    ) -> Node:
        rvec = np.asarray(rvec).reshape(3)
        tvec = np.asarray(tvec).reshape(3)

        R_cam_obj, _ = cv2.Rodrigues(rvec)

        quat = Rotation.from_matrix(R_cam_obj).as_quat()
        qx, qy, qz, qw = quat

        x, y, z = tvec

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame

        t.transform.translation.x = float(x)
        t.transform.translation.y = float(y)
        t.transform.translation.z = float(z)
        t.transform.rotation.x = float(qx)
        t.transform.rotation.y = float(qy)
        t.transform.rotation.z = float(qz)
        t.transform.rotation.w = float(qw)

        self.object_tf_brodcaster.sendTransform(t)

    def log(self, msg: str) -> None:
        self.get_logger().info(msg)


class SafeProgramControl(Node):
    def __init__(self):
        super().__init__("safe_program_control")
        self.is_remote_cli = self.create_client(
            IsInRemoteControl, "dashboard_client/is_in_remote_control"
        )
        self.cli_quit = self.create_client(Trigger, "dashboard_client/quit")
        self.cli_connect = self.create_client(Trigger, "dashboard_client/connect")
        self.play_cli = self.create_client(Trigger, "dashboard_client/play")
        self.stop_cli = self.create_client(Trigger, "dashboard_client/stop")

        for cli, name in [
            (self.is_remote_cli, "is_in_remote_control"),
            (self.cli_connect, "connect"),
            (self.play_cli, "play"),
            (self.stop_cli, "stop"),
        ]:
            while not cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f"{name} not available, waiting...")

    def reconnect_dashboard(self) -> bool:
        quit_req = Trigger.Request()
        future_quit = self.cli_quit.call_async(quit_req)
        rclpy.spin_until_future_complete(self, future_quit)

        req = Trigger.Request()
        future = self.cli_connect.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()
        self.get_logger().info(f"Reconnect: success={resp.success}'")
        return resp.success

    def _is_remote(self) -> tuple[bool, bool]:
        req = IsInRemoteControl.Request()
        future = self.is_remote_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()
        if resp is not None and resp.success:
            return True, resp.remote_control
        return False, False

    def is_remote(self) -> bool:
        call_ok, remote = self._is_remote()
        if call_ok:
            return remote

        self.get_logger().warn("is_in_remote_control failed, trying reconnect...")
        if not self.reconnect_dashboard():
            self.get_logger().error("Reconnect failed")
            return

        call_ok, remote = self._is_remote()
        if call_ok:
            return remote

        self.get_logger().error("Service call failed after reconnect")
        return False

    def play_if_remote(self) -> bool:
        if not self.is_remote():
            self.get_logger().warn("Robot not in REMOTE, not starting program")
            return
        req = Trigger.Request()
        future = self.play_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        # TODO(issue#81): Robot is playing the program but the response says success=False
        # self.get_logger().info(
        #     f"Play: {future.result().success}, {future.result().message}"
        # )

        state = self.get_program_state()
        if state == "PLAYING":
            return True
        else:
            return False

    def stop_if_remote(self) -> bool:
        if not self.is_remote():
            self.get_logger().warn("Robot not in REMOTE, not stopping program")
            return
        req = Trigger.Request()
        future = self.stop_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        # TODO(issue#81): Robot is stopping the program but the response says success=False
        # self.get_logger().info(
        #     f"Stop: {future.result().success}, {future.result().message}"
        # )

        state = self.get_program_state()
        if state == "STOPPED":
            return True
        else:
            return False

    def get_program_state(self) -> str:
        time.sleep(1)
        prog_cli = self.create_client(GetProgramState, "dashboard_client/program_state")
        while not prog_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("program_state not available, waiting...")
        req_prog = GetProgramState.Request()
        future_prog = prog_cli.call_async(req_prog)
        rclpy.spin_until_future_complete(self, future_prog)
        state = future_prog.result().state.state
        self.get_logger().info(f"Program state:: '{state}'")

        return state
