import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time
import os
import yaml
import argparse
from typing import Optional
from ament_index_python.packages import get_package_share_directory
from aegis_director.robot_director import RobotDirector


CAMERA_CONFIG = {
    "scene": {
        "pos_config": "cam_scene.yaml",
        "topic": "/cam_scene/rgb/image_rect"
    },
    "tool_front": {
        "pos_config": "cam_tool_front.yaml",
        "topic": "/cam_tool/front/image_rect"
    },
    "tool_right": {
        "pos_config": "cam_tool_right.yaml",
        "topic": "/cam_tool/right/image_rect"
    },
    "tool_left": {
        "pos_config": "cam_tool_left.yaml",
        "topic": "/cam_tool/left/image_rect"
    },
}


class ROSInterface:
    _instance: Optional["ROSInterface"] = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(ROSInterface, cls).__new__(cls)
        return cls._instance

    def __init__(self):
        if hasattr(self, "_initialized") and self._initialized:
            return
        rclpy.init()
        self.robot_director = RobotDirector(synchronous=True)
        joint_state = self.robot_director._get_joint_states()
        self.joint_names = list(joint_state.name)[1:]
        self.dof_home = {
            "shoulder_pan_joint": 0.0,
            "shoulder_lift_joint": -2.09,
            "elbow_joint": 2.09,
            "wrist_1_joint": -1.57,
            "wrist_2_joint": -1.57,
            "wrist_3_joint": 0.0,
            "robotiq_hande_left_finger_joint": 0.025,
        }
        self._initialized = True

    def move_to_home(self):
        self.robot_director.joint_move(
            joint_positions=self.dof_home, max_vel=0.5, max_accel=0.5
        )


class CalibCollectNode(Node):
    def __init__(self, image_topic, save_dir):
        super().__init__("calib_collect_node")
        self.bridge = CvBridge()
        self.image = None
        self.save_dir = save_dir
        os.makedirs(save_dir, exist_ok=True)
        self.sub = self.create_subscription(Image, image_topic, self.image_callback, 10)
        self.get_logger().info(f"Subscribed to image topic: {image_topic}")

    def image_callback(self, msg):
        try:
            self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")

    def wait_for_data(self, timeout_sec=3.0):
        start_time = time.time()
        while self.image is None and (time.time() - start_time) < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.image is not None

    def save_data(self, name, tcp_pose):
        if self.image is not None:
            image_path = os.path.join(self.save_dir, name)
            cv2.imwrite(image_path, self.image)
            self.get_logger().info(f"Saved image to: {image_path}")
            if tcp_pose is not None:
                tcp_path = os.path.splitext(image_path)[0] + ".yaml"
                with open(tcp_path, "w") as f:
                    yaml.dump({
                        "tcp_pose": {
                            "position": tcp_pose["position"].tolist(),
                            "orientation": tcp_pose["orientation"].tolist(),
                        }
                    }, f)
                self.get_logger().info(f"Saved TCP pose to: {tcp_path}")
        else:
            self.get_logger().warn("No image to save")

    def log(self, msg):
        self.get_logger().info(msg)


def load_positions(config_file_path):
    with open(config_file_path, "r") as f:
        data = yaml.safe_load(f)
    return data["positions"] if "positions" in data else data


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-c", "--camera", type=str, required=True,
        choices=CAMERA_CONFIG.keys(),
        help="Which camera: scene, tool_front, tool_right, tool_left"
    )
    args = parser.parse_args()

    package_share_path = os.path.join(get_package_share_directory("aegis_utils"))
    camera_info = CAMERA_CONFIG[args.camera]
    pos_config_path = os.path.join(package_share_path, "config", camera_info["pos_config"])
    image_topic = camera_info["topic"]
    save_dir = os.path.join("./calibration_data", args.camera)

    robot = ROSInterface()
    collect_node = CalibCollectNode(image_topic, save_dir)

    try:
        positions = load_positions(pos_config_path)

        robot.move_to_home()
        time.sleep(2)

        for i, joint_dict in enumerate(positions):
            collect_node.log(f"Moving to view position {i}")
            robot.robot_director.joint_move(
                joint_positions=joint_dict, max_vel=0.5, max_accel=0.5
            )

            time.sleep(3)

            collect_node.log(f"Capturing data at position {i}")
            if collect_node.wait_for_data():
                tcp_pose = robot.robot_director.get_tcp_pose()
                collect_node.save_data(f"{args.camera}_view_{i}.png", tcp_pose)
            else:
                collect_node.log(f"No data received at position {i}")

        collect_node.log("Moving back to home")
        robot.move_to_home()
        time.sleep(2)

    finally:
        collect_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
