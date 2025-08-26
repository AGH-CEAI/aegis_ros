import argparse
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional

import cv2
import numpy as np
import rclpy
import yaml
from aegis_director.robot_director import RobotDirector
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Time
from cv_bridge import CvBridge
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image

CAMERA_CONFIG = {
    "scene": {"pos_config": "cam_scene.yaml", "topic": "/cam_scene/rgb/image_raw"},
    "tool_front_right": {
        "pos_config": "cam_tool_front.yaml",
        "topic": "/cam_tool_front/right/image_raw",
    },
    "tool_front_left": {
        "pos_config": "cam_tool_front.yaml",
        "topic": "/cam_tool_front/left/image_raw",
    },
    "tool_right": {
        "pos_config": "cam_tool_right.yaml",
        "topic": "/cam_tool_right/image_raw",
    },
    "tool_left": {
        "pos_config": "cam_tool_left.yaml",
        "topic": "/cam_tool_left/image_raw",
    },
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-c",
        "--camera",
        type=str,
        required=True,
        choices=CAMERA_CONFIG.keys(),
        help="Which camera: scene, tool_front_right, tool_front_left, tool_right, tool_left",
    )
    parser.add_argument(
        "-p",
        "--path",
        type=str,
        default=None,
        help="Optional path to calibration data folder",
    )
    return parser.parse_args()


def load_positions(config_file_path: Path) -> List[Dict[str, float]]:
    with open(config_file_path, "r") as f:
        data = yaml.safe_load(f)
    return data.get("positions", data)


def get_timestamp(stamp: Time) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


class CalibCollectNode(Node):
    def __init__(self, robot: RobotDirector, image_topic: str, data_path: Path) -> None:
        super().__init__("calib_collect_node")
        self.robot = robot
        self.bridge = CvBridge()
        self.image = None
        self.timestamp = None
        self.mutex = threading.Lock()
        self.data_path = data_path
        self.sub = self.create_subscription(Image, image_topic, self.image_callback, 10)
        self.get_logger().info(f"Subscribed to image topic: {image_topic}")

    def move_to_home(self) -> None:
        self.robot.joint_move(
            joint_positions={
                "shoulder_pan_joint": 0.0,
                "shoulder_lift_joint": -2.09,
                "elbow_joint": 2.09,
                "wrist_1_joint": -1.57,
                "wrist_2_joint": -1.57,
                "wrist_3_joint": 0.0,
                "robotiq_hande_left_finger_joint": 0.025,
            },
            max_vel=0.5,
            max_accel=0.5,
        )

    def image_callback(self, msg: Image) -> None:
        try:
            with self.mutex:
                self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                self.timestamp = get_timestamp(msg.header.stamp)
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")

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

    def save_tcp(self, tcp_pose: Dict[str, List[float]], path: Path) -> None:
        with open(path, "w") as f:
            yaml.dump(
                {
                    "tcp_pose": {
                        "position": tcp_pose["position"].tolist(),
                        "orientation": tcp_pose["orientation"].tolist(),
                    }
                },
                f,
            )
        self.get_logger().info(f"Saved TCP pose to: {path}")

    def log(self, msg: str) -> None:
        self.get_logger().info(msg)


def collect_data(
    node: CalibCollectNode,
    robot: RobotDirector,
    positions: List[Dict[str, float]],
    data_path: Path,
    camera_name: str,
) -> None:
    node.move_to_home()
    time.sleep(2)

    for i, joint_dict in enumerate(positions):
        node.log(f"Moving to view position {i}")
        robot.joint_move(joint_positions=joint_dict, max_vel=0.5, max_accel=0.5)
        time.sleep(3)

        time_start = node.get_clock().now().nanoseconds / 1e9
        node.log("Waiting for image...")
        image = node.get_image(time_start)
        if image is not None:
            tcp_pose = robot.get_tcp_pose()
            image_path = data_path / f"{camera_name}_image_{i:02}.png"
            tcp_path = data_path / f"{camera_name}_tcp_{i:02}.yaml"
            node.save_image(image, image_path)
            node.save_tcp(tcp_pose, tcp_path)
        else:
            node.log("No image received in time")

        time.sleep(1)

    node.log("Moving back to home")
    node.move_to_home()
    time.sleep(2)


def main() -> None:
    args = parse_args()

    if args.path:
        data_path = Path(args.path).expanduser() / args.camera
    else:
        timestamp = datetime.now().strftime("%y-%m-%d_%H-%M-%S")
        data_path = Path(f"~/ceai_ws/calib_data_{timestamp}").expanduser() / args.camera

    data_path.mkdir(parents=True, exist_ok=True)

    package_share_path = Path(get_package_share_directory("aegis_utils"))
    camera_info = CAMERA_CONFIG[args.camera]
    pos_config_path = package_share_path / "config" / camera_info["pos_config"]
    image_topic = camera_info["topic"]

    rclpy.init()
    robot = RobotDirector(synchronous=True)
    collect_node = CalibCollectNode(robot, image_topic, data_path)
    executor = SingleThreadedExecutor()
    executor.add_node(collect_node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        positions = load_positions(pos_config_path)
        collect_data(collect_node, robot, positions, data_path, args.camera)
    finally:
        collect_node.destroy_node()
        rclpy.shutdown()
        executor.shutdown()
        spin_thread.join()


if __name__ == "__main__":
    main()
