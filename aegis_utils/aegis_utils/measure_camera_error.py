import argparse
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple
import sys
import termios
import tty
import math

import cv2
import numpy as np
import rclpy
from aegis_director import RobotDirector
from builtin_interfaces.msg import Time
from cv_bridge import CvBridge
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import Buffer, TransformListener


CAMERA_CONFIG = {
    "scene": {"pos_config": "cam_scene.yaml", "topic": "/cam_scene/rgb/image_raw"},
}


class CalibrationTool(Node):
    def __init__(self, tool_offset: List[float]) -> None:
        super().__init__("static_tf_broadcaster")

        # broadcaster object
        self.broadcaster = StaticTransformBroadcaster(self)

        # create TransformStamped
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "tool0"  # parent frame
        t.child_frame_id = "calibration_tool"  # child frame

        # translation (meters)
        t.transform.translation.x = tool_offset[0]
        t.transform.translation.y = tool_offset[1]
        t.transform.translation.z = tool_offset[2]

        # orientation as quaternion (x,y,z,w) # no rotation
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        # send it once
        self.broadcaster.sendTransform(t)
        self.get_logger().info("Static transform tool0 -> calibration_tool published")


class CollectImageNode(Node):
    def __init__(self, robot: RobotDirector, image_topic: str, data_path: Path) -> None:
        super().__init__("collect_image_node")
        self.robot = robot
        self.image = None
        self.timestamp = None
        self.bridge = CvBridge()
        self.data_path = data_path
        self.mutex = threading.Lock()
        self.sub = self.create_subscription(Image, image_topic, self.image_callback, 10)
        self.get_logger().info(f"Subscribed to image topic: {image_topic}")

        # tf2 setup and calibration tool
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def image_callback(self, msg: Image) -> None:
        try:
            with self.mutex:
                self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                self.timestamp = self.get_timestamp(msg.header.stamp)
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")

    def get_timestamp(self, time_msg: Time) -> float:
        return time_msg.sec + time_msg.nanosec * 1e-9

    def get_image(
        self, time_start: float, timeout: float = 3.0
    ) -> Optional[np.ndarray]:
        while self.get_clock().now().nanoseconds / 1e9 - time_start < timeout:
            with self.mutex:
                if self.image is not None and self.timestamp is not None:
                    if self.timestamp > time_start:
                        return self.image.copy()
            time.sleep(1)
        return None

    def save_image(self, image: Optional[np.ndarray], path: Path) -> None:
        if image is None:
            self.get_logger().warn("No image to save")
            return
        cv2.imwrite(str(path), image)
        self.get_logger().info(f"Saved image to: {path}")

    def get_calibration_tool_pose_in_base(self) -> Dict[str, np.ndarray]:
        """
        Returns (position, orientation) of calibration_tool in base frame.

        position: np.array([x, y, z])
        orientation: np.array([qx, qy, qz, qw])
        """
        try:
            t = self.tf_buffer.lookup_transform(
                "base",  # target frame
                "calibration_tool",  # source frame
                Time(),  # latest available
            )
        except Exception as e:
            self.get_logger().error(
                f"Failed to lookup transform base->calibration_tool: {e}"
            )
            return None, None

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

    def log(self, msg: str) -> None:
        self.get_logger().info(msg)


class MeasureCameraError:
    def __init__(
        self,
        robot: RobotDirector,
        image_node: CollectImageNode,
        camera_name: str,
        data_path: Path,
    ) -> None:
        self.robot = robot
        self.image_node = image_node
        self.camera_name = camera_name
        self.data_path = data_path
        self.errors = []

        # TODO: load from calibration file
        # self.T_cam2base, self.camera_matrix, self.dist_coeffs = self.self.load_data()
        self.T_cam2base = np.array(
            [
                [
                    0.019393463529861016,
                    0.9990240674437583,
                    -0.039683828449952906,
                    0.047714301707007856,
                ],
                [
                    0.9996413820847948,
                    -0.0186417902732842,
                    0.019224746526503706,
                    -0.34666811181735363,
                ],
                [
                    0.018466206863277983,
                    -0.04004243153875939,
                    -0.9990273283952479,
                    1.1668662643689283,
                ],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )
        self.camera_matrix = np.array(
            [
                1045.253469873161,
                0.0,
                616.7886604727472,
                0.0,
                1043.2430632943033,
                386.8198966768913,
                0.0,
                0.0,
                1.0,
            ],
            dtype=np.float32,
        ).reshape((3, 3))
        self.dist_coeffs = np.array(
            [
                0.08083301537530861,
                -0.04332893272558069,
                0.003319516691409084,
                -0.0016267198557977577,
                -0.2800871865529448,
            ],
            dtype=np.float32,
        ).reshape((5, 1))

    def move_to_home(self) -> None:
        self.robot.joint_move(
            joint_positions={
                "shoulder_pan_joint": 0.0,
                "shoulder_lift_joint": -2.09,
                "elbow_joint": 2.09,
                "wrist_1_joint": -1.57,
                "wrist_2_joint": -1.57,
                "wrist_3_joint": 0.0,
            },
            max_vel=0.5,
            max_accel=0.5,
        )

    def load_data(self) -> None:
        pass

    def working_loop(self) -> None:
        next_measure = True
        while next_measure:
            # Set robot to freedrive mode and wait for user to position the robot. Then switch back to normal mode.
            self.robot._switch_controllers(
                activate=["freedrive_mode_controller"],
                deactivate=["scaled_joint_trajectory_controller"],
            )
            time.sleep(1.0)
            self.log("""\033[93mINSTRUCTIONS::
                                (With teachpendant)
                                1) change REMOTE to LOCAL (top right corner of screen),
                                2) PAUSE the program (pause button)
                                3) With deadmen button pressed, set the end effector at the corner of the calibration board.
                                4) PLAY the program (play button)
                                5) change back to REMOTE
                                6) press ENTER at kayboard to start measuring camera error\033[93m""")
            input()
            self.robot._switch_controllers(
                activate=["scaled_joint_trajectory_controller"],
                deactivate=["freedrive_mode_controller"],
            )
            time.sleep(1.0)

            # TESTING: print joint pos
            # joint_pos = self.robot.get_joint_positions()
            # print(f"JOINT POSS:: {joint_pos}")

            tcp_pose_robot = self.image_node.get_calibration_tool_pose_in_base()
            tcp_pose_robot = tcp_pose_robot["position"].flatten().tolist()
            self.log(f"TCP pose from robot: {tcp_pose_robot}")

            self.move_to_home()
            time.sleep(1.0)  # Wait for robot to stabilize

            # image = self.get_image_from_camera()
            image = cv2.imread("error_data/test_image.png")  # For testing purpose only
            if image is None:
                next_measure = self.ask_for_next_measure()
                continue

            tcp_pose_camera_frame = self.measure_position_from_marker(image)
            tcp_pose_camera_frame = np.vstack((tcp_pose_camera_frame, [1]))

            tcp_pose_camera = self.T_cam2base @ tcp_pose_camera_frame
            tcp_pose_camera = tcp_pose_camera[:3].flatten().tolist()
            self.log(f"TCP pose from camera: {tcp_pose_camera}")

            TCP_error = self.calculate_TCP_error(tcp_pose_camera, tcp_pose_robot)
            self.errors.append(TCP_error)
            self.log(f"Error of the position:: {TCP_error[3]}")

            next_measure = self.ask_for_next_measure()

        self.analyze_results()
        self.log("\033[92mFinished measuring camera error.\033[92m")

    def get_image_from_camera(self) -> Optional[np.ndarray]:
        time_start = self.get_clock().now().nanoseconds / 1e9
        self.log("Waiting for image...")
        image = self.image_node.get_image(time_start)
        if image is not None:
            image_path = self.data_path / f"{self.camera_name}_image_{self.i:02}.png"
            self.image_node.save_image(image, image_path)
        else:
            self.image_node.log("\033[91mFailed to get image from camera.\033[91m")
        return image

    def measure_position_from_marker(self, image: np.ndarray) -> np.ndarray:
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        parameters = cv2.aruco.DetectorParameters_create()
        marker_size = 0.1415  # Marker size in meters

        corners, ids, _ = cv2.aruco.detectMarkers(
            image, aruco_dict, parameters=parameters
        )

        marker_corners = corners[0]
        image_points = marker_corners.reshape(-1, 2).astype(np.float32)

        obj_points = np.array(
            [
                [0.0, 0.0, 0.0],  # corner 0 (top-left)
                [marker_size, 0.0, 0.0],  # corner 1 (top-right)
                [marker_size, marker_size, 0.0],  # corner 2 (bottom-right)
                [0.0, marker_size, 0.0],  # corner 3 (bottom-left)
            ],
            dtype=np.float32,
        )

        retval, rvec, tvec = cv2.solvePnP(
            obj_points, image_points, self.camera_matrix, self.dist_coeffs
        )

        cv2.drawFrameAxes(
            image,
            self.camera_matrix,
            self.dist_coeffs,
            rvec,
            tvec,
            marker_size * 0.5,  # visual axis length
        )

        cv2.imshow("Test Image", image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        return tvec

    def ask_for_next_measure(self) -> bool:
        while True:
            self.log("\033[93mDo you want to measure again? (Y/n)\033[93m")
            answer = getch()
            if answer in ("Y", "\r", "\n"):
                return True
            elif answer == "n":
                return False
            else:
                self.log("Invalid input. Please press 'Y' or 'n'.")

    def analyze_results(self) -> None:
        mean_errors = np.mean([row[3] for row in self.errors])
        self.log(f"Mean error: {mean_errors}")

    def calculate_TCP_error(self, c, r) -> Tuple[float, float, float, float]:
        dx = c[0] - r[0]
        dy = c[1] - r[1]
        dz = c[2] - r[2]
        return (abs(dx), abs(dy), abs(dz), math.sqrt(dx * dx + dy * dy + dz * dz))

    def log(self, msg: str) -> None:
        self.image_node.log(msg)


def main() -> None:
    args = parse_args()
    cam_name = args.camera
    path_name = args.path
    tool_offset = args.tool_offset

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    if path_name:
        data_path = Path(path_name).expanduser() / f"{cam_name}_{timestamp}"
    else:
        data_path = (
            Path("~/ceai_ws/error_data").expanduser() / f"{cam_name}_{timestamp}"
        )

    # data_path.mkdir(parents=True, exist_ok=True)

    # package_share_path = Path(get_package_share_directory("aegis_utils"))
    camera_info = CAMERA_CONFIG[cam_name]
    # pos_config_path = package_share_path / "config" / camera_info["pos_config"]
    image_topic = camera_info["topic"]

    rclpy.init()
    robot = RobotDirector(synchronous=True)
    image_node = CollectImageNode(robot, image_topic, data_path)
    tool_tf_node = CalibrationTool(tool_offset)
    measure = MeasureCameraError(robot, image_node, cam_name, data_path)

    executor = SingleThreadedExecutor()
    executor.add_node(image_node)
    executor.add_node(tool_tf_node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        measure.working_loop()
    finally:
        executor.shutdown()
        image_node.destroy_node()
        tool_tf_node.destroy_node()
        spin_thread.join()
        rclpy.shutdown()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-t",
        "--tool_offset",
        type=float,
        nargs=3,
        default=[0.0, 0.0, 0.25],
        help="Offset of the calibration tool from tool0 frame in meters (x y z)",
    )
    parser.add_argument(
        "-c",
        "--camera",
        type=str,
        default="scene",
        choices=CAMERA_CONFIG.keys(),
        help="Which camera: scene, tool_front_right, tool_front_left, tool_right, tool_left",
    )
    parser.add_argument(
        "-p",
        "--path",
        type=str,
        default=None,
        help="Optional path to results data folder",
    )
    return parser.parse_args()


def getch():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch


if __name__ == "__main__":
    main()
