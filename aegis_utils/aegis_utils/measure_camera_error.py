import argparse
import math
import sys
import termios
import textwrap
import threading
import time
import tty
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import rclpy
import yaml
from rclpy.executors import SingleThreadedExecutor

from aegis_director import RobotDirector
from aegis_utils.measure_camera_error_ros_nodes import (
    CalibrationTool,
    CollectImageNode,
    SafeProgramControl,
    TcpPosCamera,
)


CAMERA_CONFIG = {
    "scene": {"pos_config": "cam_scene.yaml", "topic": "/cam_scene/rgb/image_raw"},
}

RESULTS_FILENAME = "results.yaml"


class MeasureCameraError:
    def __init__(
        self,
        robot: RobotDirector,
        image_node: CollectImageNode,
        camera_name: str,
        res_path: Path,
        data_path: Path,
        aruco_dict: cv2.aruco_Dictionary,
        marker_size: float,
    ) -> None:
        self.robot = robot
        self.image_node = image_node
        self.camera_name = camera_name
        self.res_path = res_path
        self.data_path = data_path
        self.errors = []
        self.iteration = 0
        self.aruco_dict = aruco_dict
        self.marker_size = marker_size
        self.camera_matrix, self.dist_coeffs = self.load_data()
        self.safe_program_control = SafeProgramControl()

    def destroy(self):
        if self.safe_program_control is not None:
            self.safe_program_control.destroy_node()
            self.safe_program_control = None

    # TODO(issue#80) Get the home position from the SRDF file
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

    def load_data(self) -> tuple[np.ndarray, np.ndarray]:
        intrinsics_path = self.data_path / f"{self.camera_name}_intrinsics.yaml"
        with intrinsics_path.open("r") as f:
            intrinsics = yaml.safe_load(f)
        camera_matrix = np.array(
            intrinsics["camera_matrix"]["data"], dtype=np.float32
        ).reshape((3, 3))
        dist_coeffs = np.array(
            intrinsics["distortion_coefficients"]["data"], dtype=np.float32
        ).reshape(
            (
                intrinsics["distortion_coefficients"]["cols"],
                intrinsics["distortion_coefficients"]["rows"],
            )
        )

        return camera_matrix, dist_coeffs

    def working_loop(self) -> None:
        next_measure = True
        while next_measure:
            self.robot._switch_controllers(
                activate=["freedrive_mode_controller"],
                deactivate=["scaled_joint_trajectory_controller"],
            )
            time.sleep(0.5)
            while not self.safe_program_control.is_remote():
                self.log("Waiting for REMOTE control...")
                time.sleep(1)

            while not self.safe_program_control.stop_if_remote():
                self.log("Program has not stopped, trying one more time...")

            self.log(
                textwrap.dedent("""\
                    \033[93mINSTRUCTIONS::
                        (With teach pendant)
                        1) Change REMOTE to LOCAL (top-right corner of screen)
                        2) With deadman button pressed, set the end effector at the corner of the calibration board
                        3) Change back to REMOTE
                        4) Press ENTER on the keyboard to start measuring camera error\033[93m
            """)
            )
            input()

            while not self.safe_program_control.is_remote():
                self.log("Waiting for REMOTE control...")
                time.sleep(1)

            self.safe_program_control.reconnect_dashboard()
            while not self.safe_program_control.play_if_remote():
                self.log("Program has not started, trying one more time...")

            self.robot._switch_controllers(
                activate=["scaled_joint_trajectory_controller"],
                deactivate=["freedrive_mode_controller"],
            )
            time.sleep(2.0)

            tcp_pose_robot = self.image_node.get_calibration_tool_pose_in_base()
            tcp_pose_robot = tcp_pose_robot["position"].flatten().tolist()

            self.move_to_home()
            time.sleep(1.0)

            image = self.get_image_from_camera()
            if image is None:
                next_measure = self.ask_for_next_measure()
                continue

            tcp_pos_in_camera_frame = self.measure_position_from_marker(image)
            if tcp_pos_in_camera_frame is None:
                next_measure = self.ask_for_next_measure()
                continue
            else:
                self.image_node.last_object_tf = (
                    tcp_pos_in_camera_frame.rvec,
                    tcp_pos_in_camera_frame.tvec,
                )

                time.sleep(1.0)
                tcp_pos_in_camera_frame.tvec = np.vstack(
                    (tcp_pos_in_camera_frame.tvec, [1])
                )

                detect_object_pos = self.image_node.get_object_pose_in_base()
                detect_object_pos = detect_object_pos["position"].flatten().tolist()

                self.log(f"TCP pose from camera: {detect_object_pos}")
                self.log(f"TCP pose from robot: {tcp_pose_robot}")

                TCP_error = self.calculate_TCP_error(detect_object_pos, tcp_pose_robot)
                self.errors.append(TCP_error)
                self.log(f"Error of the position:: {TCP_error[3]}")

            next_measure = self.ask_for_next_measure()

        self.analyze_results(self.errors, RESULTS_FILENAME)
        self.log("\033[92mFinished measuring camera error.\033[92m")

    def get_image_from_camera(self) -> np.ndarray | None:
        time_start = self.image_node.get_clock().now().nanoseconds / 1e9
        self.log("Waiting for image...")
        image = self.image_node.get_image(time_start)
        if image is not None:
            image_path = (
                self.res_path / f"{self.camera_name}_image_{self.iteration:02}.png"
            )
            self.image_node.save_image(image, image_path)
            self.iteration += 1
        else:
            self.image_node.log("\033[91mFailed to get image from camera.\033[91m")
        return image

    def measure_position_from_marker(self, image: np.ndarray) -> TcpPosCamera | None:
        parameters = cv2.aruco.DetectorParameters()

        corners, ids, _ = cv2.aruco.detectMarkers(
            image, self.aruco_dict, parameters=parameters
        )

        if len(corners) == 0:
            self.log("No detected marker at image")
            return None

        marker_corners = corners[0]
        image_points = marker_corners.reshape(-1, 2).astype(np.float32)

        obj_points = np.array(
            [
                [0.0, 0.0, 0.0],
                [self.marker_size, 0.0, 0.0],
                [self.marker_size, self.marker_size, 0.0],
                [0.0, self.marker_size, 0.0],
            ],
            dtype=np.float32,
        )

        retval, rvec, tvec = cv2.solvePnP(
            obj_points, image_points, self.camera_matrix, self.dist_coeffs
        )

        return TcpPosCamera(retval=retval, rvec=rvec, tvec=tvec)

    def calculate_TCP_error(
        self, c: np.ndarray, r: np.ndarray
    ) -> tuple[float, float, float, float]:
        dx = c[0] - r[0]
        dy = c[1] - r[1]
        dz = c[2] - r[2]
        return (abs(dx), abs(dy), abs(dz), math.sqrt(dx * dx + dy * dy + dz * dz))

    def analyze_results(self, errors: list, file_name: str) -> None:
        if not errors:
            self.log("No errors collected; nothing to analyze/save.")
            return

        errors_np = np.asarray(errors, dtype=float)

        if errors_np.ndim != 2 or errors_np.shape[1] != 4:
            raise ValueError(f"'errors' must have shape (N, 4). Got {errors_np.shape}")

        errors_x, errors_y, errors_z, errors_all = errors_np.T

        mean_errors = {
            "x": float(errors_x.mean()),
            "y": float(errors_y.mean()),
            "z": float(errors_z.mean()),
            "total": float(errors_all.mean()),
        }

        std_errors = {
            "x": float(errors_x.std()),
            "y": float(errors_y.std()),
            "z": float(errors_z.std()),
            "total": float(errors_all.std()),
        }

        data_to_save = {
            "errors": {
                "x": errors_x.tolist(),
                "y": errors_y.tolist(),
                "z": errors_z.tolist(),
                "total": errors_all.tolist(),
            },
            "statistics": {
                "mean": mean_errors,
                "std_dev": std_errors,
            },
        }

        yaml_file = self.res_path / file_name
        with open(yaml_file, "w") as f:
            yaml.safe_dump(data_to_save, f, default_flow_style=False, sort_keys=False)

        self.log(f"Mean error: {mean_errors}")
        self.log(f"STD: {std_errors}")
        self.log(f"Saved YAML to: {yaml_file}")

    def ask_for_next_measure(self) -> bool:
        while True:
            self.log("\033[93mDo you want to measure again? (Y/n)\033[93m")
            answer = get_character()
            if answer in ("Y", "\r", "\n"):
                return True
            elif answer == "n":
                return False
            else:
                self.log("Invalid input. Please press 'Y' or 'n'.")

    def log(self, msg: str) -> None:
        self.image_node.log(msg)


def main() -> None:
    args = parse_args()
    cam_name = args.camera
    result_path = args.results_path
    data_path = args.data_path
    tool_offset = args.tool_offset

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    if result_path:
        res_path = Path(result_path).expanduser() / f"{cam_name}_{timestamp}"
    else:
        res_path = Path("~/ceai_ws/error_data").expanduser() / f"{cam_name}_{timestamp}"

    if data_path:
        data_path = Path(data_path).expanduser()
    else:
        data_path = Path("~/ceai_ws/src/aegis_ros/aegis_utils/config").expanduser()

    res_path.mkdir(parents=True, exist_ok=True)
    camera_info = CAMERA_CONFIG[cam_name]
    image_topic = camera_info["topic"]
    board_path = data_path / "charuco_board.yaml"

    with open(board_path, "r") as f:
        board_cfg = yaml.safe_load(f)
    board_params = board_cfg["calib"]
    marker_size = board_params["marker_size"]
    aruco_dict = cv2.aruco.getPredefinedDictionary(
        getattr(cv2.aruco, board_params["aruco_dict"])
    )

    rclpy.init()
    robot = RobotDirector(synchronous=True)
    image_node = CollectImageNode(robot, image_topic, res_path)
    tool_tf_node = CalibrationTool(tool_offset)
    measure = MeasureCameraError(
        robot, image_node, cam_name, res_path, data_path, aruco_dict, marker_size
    )

    executor = SingleThreadedExecutor()
    executor.add_node(image_node)
    executor.add_node(tool_tf_node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        measure.working_loop()
    finally:
        measure.destroy()
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
        default=[0.00077, 0.00053, 0.26455],
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
        "-r",
        "--results-path",
        type=str,
        default=None,
        help="Optional path to results data directory",
    )
    parser.add_argument(
        "-d",
        "--data-path",
        type=str,
        default=None,
        help="Optional path to input data directory",
    )
    return parser.parse_args()


def get_character():
    """
    Read a single character from standard input without waiting for Enter key.
    """
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    ch = None
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch


if __name__ == "__main__":
    main()
