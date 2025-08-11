import argparse
import os
import threading
import time
from typing import Optional

import cv2
import yaml
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory
from aegis_director.robot_director import RobotDirector


CAMERA_CONFIG = {
    "scene": {"pos_config": "cam_scene.yaml", "topic": "/cam_scene/rgb/image_rect"},
    "tool_front_right": {
        "pos_config": "cam_tool_front.yaml",
        "topic": "/cam_tool/right/image_rect",
    },
    "tool_front_left": {
        "pos_config": "cam_tool_front.yaml",
        "topic": "/cam_tool/left/image_rect",
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


def load_positions(config_file_path):
    with open(config_file_path, "r") as f:
        data = yaml.safe_load(f)
    return data.get("positions", data)


def get_timestamp(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class CalibCollectNode(Node):
    def __init__(self, robot, image_topic, data_dir):
        super().__init__("calib_collect_node")
        self.robot = robot
        self.bridge = CvBridge()
        self.image = None
        self.timestamp = None
        self.mutex = threading.Lock()
        self.data_dir = data_dir
        self.sub = self.create_subscription(Image, image_topic, self.image_callback, 10)
        self.get_logger().info(f"Subscribed to image topic: {image_topic}")

    def move_to_home(self):
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
            max_accel=0.5
        )

    def image_callback(self, msg):
        try:
            with self.mutex:
                self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                self.timestamp = get_timestamp(msg.header.stamp)
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")

    def get_image(self, time_start, timeout=3.0):
        while self.get_clock().now().nanoseconds / 1e9 - time_start < timeout:
            with self.mutex:
                if self.image is not None and self.timestamp is not None:
                    if self.timestamp > time_start:
                        return self.image.copy()
            time.sleep(1)
        return None

    def save_image(self, image, path):
        if image is None:
            self.get_logger().warn("No image to save")
            return
        cv2.imwrite(path, image)
        self.get_logger().info(f"Saved image to: {path}")

    def save_tcp(self, tcp_pose, path):
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

    def log(self, msg):
        self.get_logger().info(msg)


def collect_data(
    node: CalibCollectNode, robot, positions, data_dir, camera_name
):
    node.move_to_home()
    time.sleep(2)

    for i, joint_dict in enumerate(positions):
        node.log(f"Moving to view position {i}")
        robot.joint_move(
            joint_positions=joint_dict, max_vel=0.5, max_accel=0.5
        )
        time.sleep(3)

        time_start = node.get_clock().now().nanoseconds / 1e9
        node.log("Waiting for image...")
        image = node.get_image(time_start)
        if image is not None:
            tcp_pose = robot.get_tcp_pose()
            img_path = os.path.join(data_dir, f"{camera_name}_view_{i}.png")
            tcp_path = os.path.splitext(img_path)[0] + ".yaml"
            node.save_image(image, img_path)
            node.save_tcp(tcp_pose, tcp_path)
        else:
            node.log("No image received in time")

        time.sleep(1)

    node.log("Moving back to home")
    node.move_to_home()
    time.sleep(2)


def main():
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
    args = parser.parse_args()

    package_share_path = os.path.join(get_package_share_directory("aegis_utils"))
    camera_info = CAMERA_CONFIG[args.camera]
    pos_config_path = os.path.join(
        package_share_path, "config", camera_info["pos_config"]
    )
    image_topic = camera_info["topic"]

    if args.path:
        data_dir = os.path.join(os.path.expanduser(args.path), args.camera)
    else:
        data_dir = os.path.expanduser(f"~/ceai_ws/calibration_data/{args.camera}")

    os.makedirs(data_dir, exist_ok=True)

    rclpy.init()
    robot = RobotDirector(synchronous=True)
    collect_node = CalibCollectNode(robot, image_topic, data_dir)
    executor = SingleThreadedExecutor()
    executor.add_node(collect_node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        positions = load_positions(pos_config_path)
        collect_data(collect_node, robot, positions, data_dir, args.camera)
    finally:
        collect_node.destroy_node()
        rclpy.shutdown()
        executor.shutdown()
        spin_thread.join()


if __name__ == "__main__":
    main()
