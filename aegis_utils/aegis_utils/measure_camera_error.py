import argparse
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional
import sys
import termios
import tty

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
        "topic": "/cam_tool_right/image_color",
    },
    "tool_left": {
        "pos_config": "cam_tool_left.yaml",
        "topic": "/cam_tool_left/image_color",
    },
}


class CollectImageNode(Node):
    def __init__(self, robot: RobotDirector, image_topic: str, data_path: Path) -> None:
        super().__init__('collect_image_node')
        self.robot = robot
        self.image = None
        self.timestamp = None
        self.bridge = CvBridge()
        self.data_path = data_path
        self.mutex = threading.Lock()
        self.sub = self.create_subscription(Image, image_topic, self.image_callback, 10)
        self.get_logger().info(f"Subscribed to image topic: {image_topic}")
        print("I am here, before moving to home")

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

    def log(self, msg: str) -> None:
        self.get_logger().info(msg)



class MeasureCameraErrorNode(Node):
    def __init__(self,  robot: RobotDirector, image_node: CollectImageNode, camera_name: str, data_path: Path) -> None:
        super().__init__('measure_camera_error_node')
        self.camera_name = camera_name
        self.data_path = data_path
        self.robot = robot
        self.image_node = image_node

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
    
    def working_loop(self) -> None:
        next_measure = True
        while next_measure:
            # TODO: change the way to control the robot. Make it possible to move the robot manually

            self.log("\033[93mSet the end effector at the corner of the calibration board.\033[93m")
            self.log("\033[93mWhen robot is in position, press ENTER to start measuring camera error...\033[93m ")
            input()

            # TODO: change the way to control the robot. Control from remote

            # get TCP pose of specific pointer
            tcp_pose_robot = self.robot.get_tcp_pose() 
            
            # move robot to home position
            self.move_to_home()
            time.sleep(1.0)  # Wait for robot to stabilize

            # get image from camera
            time_start = self.get_clock().now().nanoseconds / 1e9
            self.log("Waiting for image...")
            image = self.image_node.get_image(time_start)
            if image is not None:
                image_path = self.data_path / f"{self.camera_name}_image_{self.i:02}.png"
                self.image_node.save_image(image, image_path)
            else: 
                self.image_node.log("\033[91mFailed to get image from camera.\033[91m")
                next_measure = self.ask_for_next_measure()
                continue

            # TODO: measure TCP pos from image
            tcp_pose_camera = None 


            # TODO: calculate error


            # TODO: save TCP pos data and error data
            tcp_path = self.data_path / f"{self.camera_name}_tcp_{self.i:02}.yaml"
            self.save_tcp(tcp_pose_robot, tcp_path)

            next_measure = self.ask_for_next_measure()

        self.analyze_results()
        self.log("\033[92mFinished measuring camera error.\033[92m")
        

    def save_tcp(self, 
        tcp_pose_robot: Dict[str, List[float]], 
        tcp_pose_camera: Dict[str, List[float]], 
        path: Path
    ) -> None:
        with open(path, "w") as f:
            yaml.dump(
                {
                    "tcp_pose_robot": {
                        "position": tcp_pose_robot["position"].tolist(),
                        "orientation": tcp_pose_robot["orientation"].tolist(),
                    }
                    "tcp_pose_camera": {
                        "position": tcp_pose_camera["position"].tolist(),
                        "orientation": tcp_pose_camera["orientation"].tolist(),
                    }
                },
                f,
            )
        self.get_logger().info(f"Saved TCP pose to: {path}")


    def ask_for_next_measure(self) -> bool:
            while True:
                self.log("\033[93mDo you want to measure again? (Y/n)\033[93m")
                answer = getch()
                if answer == 'Y':
                    return True
                elif answer == 'n':
                    return False
                else:
                    self.log("Invalid input. Please press 'Y' or 'n'.")

    def analyze_results(self) -> None:
        pass

    def log(self, msg: str) -> None:
        self.get_logger().info(msg)



def main() -> None:
    args = parse_args()
    cam_name = args.camera  
    path_name = args.path

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    if path_name:
        data_path = Path(path_name).expanduser() / f"{cam_name}_{timestamp}"
    else:
        data_path = (
            Path("~/ceai_ws/error_data").expanduser() / f"{cam_name}_{timestamp}"
        )
    
    data_path.mkdir(parents=True, exist_ok=True)

    package_share_path = Path(get_package_share_directory("aegis_utils"))
    camera_info = CAMERA_CONFIG[cam_name]
    pos_config_path = package_share_path / "config" / camera_info["pos_config"]
    image_topic = camera_info["topic"]

    rclpy.init()
    robot = RobotDirector(synchronous=True)
    image_node = CollectImageNode(robot, image_topic, data_path)
    measure_node = MeasureCameraErrorNode(robot, image_node, cam_name, data_path)
    executor = SingleThreadedExecutor()
    executor.add_node(image_node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        measure_node.working_loop()
    finally:
        measure_node.destroy_node()
        rclpy.shutdown()
        executor.shutdown()
        spin_thread.join()



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