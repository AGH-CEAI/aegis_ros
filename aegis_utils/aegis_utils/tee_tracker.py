"""
Tracks the push_t tee on the table with the scene camera: two ArUco markers on its top face.

Each marker corner is projected onto the plane of the tee top (a ray-plane intersection with the
calibrated camera pose) instead of trusting the PnP depth, which at 1.1 m is the noisiest part of
a single-marker pose. The planar pose (x, y, yaw of the tee origin, crossbar centre on the table)
is published as a PoseStamped and sent over UDP to aegis_gym, which has no ROS. aegis_gym sends
the goal and the requested spawn pose back, drawn on the debug image for the operator.

The marker sheet comes from `tee_marker_sheet.py`; its layout is the `MARKERS` table below.
"""

import json
import math
import socket
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

# marker id -> centre in the tee frame [m]; the crossbar along x, the stem towards -y
MARKERS = {0: (0.0, 0.0), 1: (0.0, -0.13)}
MARKER_SIZE = 0.036  # [m] black square side
ARUCO_DICT = "DICT_4X4_50"
TEE_HEIGHT = 0.04
# tee footprint in its frame (corners, closed), for the debug drawing
TEE_OUTLINE = np.array(
    [
        (-0.1, 0.025),
        (0.1, 0.025),
        (0.1, -0.025),
        (0.025, -0.025),
        (0.025, -0.175),
        (-0.025, -0.175),
        (-0.025, -0.025),
        (-0.1, -0.025),
    ]
)


def _make_detector(dict_name: str):
    """OpenCV >= 4.7 has ArucoDetector; the ROS image ships 4.5 with the function API."""
    aruco = cv2.aruco
    dictionary = aruco.getPredefinedDictionary(getattr(aruco, dict_name))
    if hasattr(aruco, "ArucoDetector"):
        params = aruco.DetectorParameters()
        params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
        detector = aruco.ArucoDetector(dictionary, params)
        return lambda gray: detector.detectMarkers(gray)[:2]
    params = aruco.DetectorParameters_create()
    params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
    return lambda gray: aruco.detectMarkers(gray, dictionary, parameters=params)[:2]


def tee_pose_from_markers(
    corners_xy: dict[int, np.ndarray],
) -> tuple[float, float, float] | None:
    """
    Tee origin (x, y, yaw) from the table-plane corners [4, 2] of the visible markers (ArUco
    order: top-left, top-right, bottom-right, bottom-left; the marker top points to tee +y).
    With both markers the yaw comes from the 13 cm line between their centres.
    """
    estimates = []
    for marker_id, c in corners_xy.items():
        centre = c.mean(axis=0)
        x_axis = (c[1] + c[2]) - (c[0] + c[3])
        yaw = math.atan2(x_axis[1], x_axis[0])
        estimates.append((marker_id, centre, yaw))
    if not estimates:
        return None
    if len(estimates) >= 2:
        (_, c0, _), (_, c1, _) = sorted(estimates, key=lambda e: e[0])[:2]
        d = np.subtract(
            MARKERS[0], MARKERS[1]
        )  # stem marker -> crossbar marker, tee frame
        v = c0 - c1
        yaw = math.atan2(v[1], v[0]) - math.atan2(d[1], d[0])
    else:
        yaw = estimates[0][2]
    rot = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
    origins = [centre - rot @ np.asarray(MARKERS[i]) for i, centre, _ in estimates]
    x, y = np.mean(origins, axis=0)
    return float(x), float(y), math.atan2(math.sin(yaw), math.cos(yaw))


class TeeTracker(Node):
    def __init__(self):
        super().__init__("tee_tracker")
        p = self.declare_parameter
        self._image_topic = p("image_topic", "/cam_scene/rgb/image_rect").value
        self._info_topic = p("camera_info_topic", "/cam_scene/rgb/camera_info").value
        self._base_frame = p("base_frame", "base_link").value
        self._cam_frame = p(
            "camera_frame", "cam_scene_rgb_camera_optical_frame_cal"
        ).value
        default_extrinsics = (
            Path(__file__).resolve().parent.parent / "config" / "scene_extrinsics.yaml"
        )
        self._extrinsics_file = p("extrinsics_file", str(default_extrinsics)).value
        # the sim table top is at z = 0.818 - 0.821 m in base_link (env.yaml table/workbench)
        self._table_z = p("table_top_z", -0.003).value
        self._sheet_thickness = p("sheet_thickness", 0.0002).value
        self._gym_address = (
            p("gym_host", "127.0.0.1").value,
            p("gym_port", 5601).value,
        )
        self._listen_port = p("listen_port", 5602).value
        self._debug_scale = p("debug_image_scale", 0.5).value

        self._bridge = CvBridge()
        self._detect = _make_detector(ARUCO_DICT)
        self._K_inv: np.ndarray | None = None
        self._T_base_cam: np.ndarray | None = None
        self._overlay: dict[str, tuple[float, float, float]] = {}
        self._last_pose: tuple[float, float, float] | None = None
        self._latencies: list[float] = []
        self._frames = 0
        self._misses = 0

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._rx.bind(("0.0.0.0", self._listen_port))
        self._rx.setblocking(False)

        self._pose_pub = self.create_publisher(PoseStamped, "~/pose", 10)
        self._image_pub = self.create_publisher(Image, "~/image", 1)
        self.create_subscription(CameraInfo, self._info_topic, self._on_info, 1)
        self.create_subscription(
            Image, self._image_topic, self._on_image, qos_profile_sensor_data
        )
        self.create_timer(5.0, self._report)
        self.get_logger().info(
            f"Tracking markers {list(MARKERS)} ({ARUCO_DICT}, {MARKER_SIZE * 1e3:.0f} mm) on "
            f"{self._image_topic}; UDP to {self._gym_address}, listening on {self._listen_port}"
        )

    def _on_info(self, msg: CameraInfo) -> None:
        # image_rect is undistorted: its intrinsics are the projection matrix, not K
        P = np.array(msg.p, dtype=float).reshape(3, 4)
        self._K_inv = np.linalg.inv(P[:, :3])

    def _camera_pose(self) -> np.ndarray | None:
        if self._T_base_cam is not None:
            return self._T_base_cam
        try:
            tf = self._tf_buffer.lookup_transform(
                self._base_frame, self._cam_frame, rclpy.time.Time()
            ).transform
            T = np.eye(4)
            T[:3, :3] = Rotation.from_quat(
                [tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w]
            ).as_matrix()
            T[:3, 3] = [tf.translation.x, tf.translation.y, tf.translation.z]
            source = f"TF {self._base_frame} <- {self._cam_frame}"
        except Exception:  # noqa: BLE001, the TF is optional: the cameras launch may not publish it
            with open(self._extrinsics_file) as f:
                T = np.array(yaml.safe_load(f)["T_base2cam"], dtype=float)
            source = self._extrinsics_file
        self._T_base_cam = T
        self.get_logger().info(f"Camera pose from {source}:\n{np.round(T, 4)}")
        return T

    def _to_table_plane(self, pixels: np.ndarray, T: np.ndarray) -> np.ndarray:
        """[N, 2] pixels -> [N, 2] base-frame xy on the plane of the marker sheet."""
        z = self._table_z + TEE_HEIGHT + self._sheet_thickness
        rays = (self._K_inv @ np.c_[pixels, np.ones(len(pixels))].T).T @ T[:3, :3].T
        origin = T[:3, 3]
        t = (z - origin[2]) / rays[:, 2]
        return origin[:2] + t[:, None] * rays[:, :2]

    def _to_pixels(self, xy: np.ndarray, T: np.ndarray) -> np.ndarray:
        z = self._table_z + TEE_HEIGHT
        p_base = np.c_[xy, np.full(len(xy), z)]
        p_cam = (p_base - T[:3, 3]) @ T[:3, :3]
        uvw = p_cam @ np.linalg.inv(self._K_inv).T
        return uvw[:, :2] / uvw[:, 2:3]

    def _poll_overlay(self) -> None:
        while True:
            try:
                data, _ = self._rx.recvfrom(4096)
            except BlockingIOError:
                return
            msg = json.loads(data)
            self._overlay[msg["kind"]] = (msg["x"], msg["y"], msg["yaw"])
            if msg["kind"] == "spawn":
                x, y, yaw = self._overlay["spawn"]
                self.get_logger().info(
                    f"Place the tee: x {x:.3f} m, y {y:.3f} m, yaw {math.degrees(yaw):.0f} deg"
                )

    def _on_image(self, msg: Image) -> None:
        if self._K_inv is None:
            return
        T = self._camera_pose()
        self._poll_overlay()
        img = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners, ids = self._detect(gray)
        found = {}
        if ids is not None:
            for c, i in zip(corners, ids.flatten()):
                if int(i) in MARKERS:
                    found[int(i)] = self._to_table_plane(c.reshape(4, 2), T)
        pose = tee_pose_from_markers(found)
        self._frames += 1
        stamp = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        if pose is None:
            self._misses += 1
        else:
            self._last_pose = pose
            latency = self.get_clock().now().nanoseconds * 1e-9 - stamp
            self._latencies.append(latency)
            self._publish(pose, msg, latency, n_markers=len(found))
        if self._image_pub.get_subscription_count() > 0:
            self._publish_debug(img, corners, ids, pose, T, msg.header)

    def _publish(self, pose, msg: Image, latency: float, n_markers: int) -> None:
        x, y, yaw = pose
        out = PoseStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self._base_frame
        out.pose.position.x, out.pose.position.y, out.pose.position.z = (
            x,
            y,
            self._table_z,
        )
        out.pose.orientation.z = math.sin(yaw / 2)
        out.pose.orientation.w = math.cos(yaw / 2)
        self._pose_pub.publish(out)
        packet = {
            "x": x,
            "y": y,
            "yaw": yaw,
            "z": self._table_z,
            "latency": latency,
            "n_markers": n_markers,
            "wall_time": time.time(),
        }
        self._tx.sendto(json.dumps(packet).encode(), self._gym_address)

    def _publish_debug(self, img, corners, ids, pose, T, header) -> None:
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(img, corners, ids)

        def outline(p, color):
            x, y, yaw = p
            rot = np.array(
                [[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]]
            )
            px = self._to_pixels(TEE_OUTLINE @ rot.T + [x, y], T).astype(np.int32)
            cv2.polylines(img, [px], True, color, 2)

        for kind, color in (("goal", (0, 0, 255)), ("spawn", (0, 200, 255))):
            if kind in self._overlay:
                outline(self._overlay[kind], color)
        if pose is not None:
            outline(pose, (0, 255, 0))
            x, y, yaw = pose
            text = f"x {x:.3f} y {y:.3f} yaw {math.degrees(yaw):.1f}"
            cv2.putText(
                img, text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2
            )
        small = cv2.resize(img, None, fx=self._debug_scale, fy=self._debug_scale)
        out = self._bridge.cv2_to_imgmsg(small, encoding="bgr8")
        out.header = header
        self._image_pub.publish(out)

    def _report(self) -> None:
        if self._frames == 0:
            self.get_logger().warning(f"No images on {self._image_topic} yet")
            return
        lat = np.array(self._latencies) * 1e3 if self._latencies else np.array([np.nan])
        pose = (
            "none"
            if self._last_pose is None
            else f"x {self._last_pose[0]:.3f} y {self._last_pose[1]:.3f} yaw {math.degrees(self._last_pose[2]):.1f} deg"
        )
        self.get_logger().info(
            f"{self._frames / 5.0:.1f} fps, tee missing in {self._misses}/{self._frames} frames, "
            f"stamp-to-pose latency median {np.median(lat):.0f} ms (max {lat.max():.0f}); "
            f"last pose {pose}"
        )
        self._frames, self._misses, self._latencies = 0, 0, []


def main():
    rclpy.init()
    node = TeeTracker()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
