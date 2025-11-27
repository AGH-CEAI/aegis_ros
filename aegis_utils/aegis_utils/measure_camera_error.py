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
        self.T_cam2base = np.array([
            [0.019393463529861016, 0.9990240674437583, -0.039683828449952906, 0.047714301707007856],
            [0.9996413820847948,  -0.0186417902732842, 0.019224746526503706, -0.34666811181735363],
            [0.018466206863277983, -0.04004243153875939, -0.9990273283952479, 1.1668662643689283],
            [0.0, 0.0, 0.0, 1.0]
        ])
        self.camera_matrix = np.array([1045.253469873161, 0.0, 616.7886604727472, 0.0, 1043.2430632943033, 386.8198966768913, 0.0, 0.0, 1.0], dtype=np.float32).reshape((3, 3))
        self.dist_coeffs = np.array([0.08083301537530861, -0.04332893272558069, 0.003319516691409084, -0.0016267198557977577, -0.2800871865529448], dtype=np.float32).reshape((5, 1))


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
            # TODO: check; change the way to control the robot. Make it possible to move the robot manually
            self.robot._switch_controllers(
                activate=["freedrive_mode_controller"],
                deactivate=["scaled_joint_trajectory_controller"],
            )
            time.sleep(1.0)

            self.log("\033[93mSet the end effector at the corner of the calibration board.\033[93m")
            self.log("\033[93mWhen robot is in position, press ENTER to start measuring camera error...\033[93m ")
            input()            
            
            # TODO: check; change the way to control the robot. Control from remote
            self.robot._switch_controllers(
                activate=["scaled_joint_trajectory_controller"],
                deactivate=["freedrive_mode_controller"],
            )
            time.sleep(1.0)

            # TODO: get TCP pose of specific pointer
            tcp_pose_robot = self.robot.get_tcp_pose()["position"]
            print(f"TCP pose from robot: {tcp_pose_robot}")
            
            # move robot to home position
            self.move_to_home()
            time.sleep(1.0)  # Wait for robot to stabilize

            # get image from camera
            # time_start = self.get_clock().now().nanoseconds / 1e9
            # self.log("Waiting for image...")
            # image = self.image_node.get_image(time_start)
            # if image is not None:
            #     image_path = self.data_path / f"{self.camera_name}_image_{self.i:02}.png"
            #     self.image_node.save_image(image, image_path)
            # else: 
            #     self.image_node.log("\033[91mFailed to get image from camera.\033[91m")
            #     next_measure = self.ask_for_next_measure()
            #     continue
            image  = cv2.imread("error_data/test_image.png")  # For testing purpose only
            if image is None:
                next_measure = self.ask_for_next_measure()
                continue


            # TODO: measure TCP pos from image                
            tcp_pose_camera_frame = self.measure_position_from_marker(image)
            tcp_pose_camera_frame = np.vstack((tcp_pose_camera_frame, [1]))  # Convert to homogeneous coordinates

            tcp_pose_camera = self.T_cam2base @ tcp_pose_camera_frame
            print(f"TCP pose from camera: {tcp_pose_camera[:3].flatten().tolist()}")

            print(f"TCP pose robot: {tcp_pose_robot}")

            # TODO: calculate error
            error = np.array(tcp_pose_robot) - tcp_pose_camera[:3].flatten()
            print(f"Position error (m): {error.tolist()}")

            # TODO: save TCP pos data and error data

            next_measure = self.ask_for_next_measure()

        self.analyze_results()
        self.log("\033[92mFinished measuring camera error.\033[92m")
        

    def measure_position_from_marker(
        self, image: np.ndarray
    ) -> np.ndarray:
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        parameters = cv2.aruco.DetectorParameters_create()
        marker_size = 0.1415  # Marker size in meters

        corners, ids, _ = cv2.aruco.detectMarkers(image, aruco_dict, parameters=parameters)

        marker_corners = corners[0]
        image_points = marker_corners.reshape(-1, 2).astype(np.float32)

        obj_points = np.array([
            [0.0,           0.0,            0.0],           # corner 0 (top-left)
            [marker_size, 0.0,            0.0],           # corner 1 (top-right)
            [marker_size, marker_size,  0.0],           # corner 2 (bottom-right)
            [0.0,           marker_size,  0.0],           # corner 3 (bottom-left)
        ], dtype=np.float32)

        retval, rvec, tvec = cv2.solvePnP(
            obj_points,
            image_points,
            self.camera_matrix,
            self.dist_coeffs
        )

        cv2.drawFrameAxes(
            image,
            self.camera_matrix,
            self.dist_coeffs,
            rvec,
            tvec,
            marker_size * 0.5  # visual axis length
        )

        cv2.imshow("Test Image", image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        return tvec


    def save_tcp(self, 
        tcp_pose_robot: Dict[str, List[float]], 
        tcp_pose_camera: Dict[str, List[float]], 
        path: Path
    ) -> None:
        pass


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