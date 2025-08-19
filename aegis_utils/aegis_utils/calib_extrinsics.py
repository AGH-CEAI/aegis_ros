import argparse
import json
from pathlib import Path
from typing import Tuple

import cv2
import numpy as np
import yaml
from natsort import natsorted
from scipy.spatial.transform import Rotation as R

CAMERA_CONFIG = {
    "scene": {},
    "tool_front_right": {},
    "tool_front_left": {},
    "tool_right": {},
    "tool_left": {},
}


def load_tcp(tcp_path: Path) -> Tuple[np.ndarray, np.ndarray]:
    with open(tcp_path, "r") as f:
        data = yaml.safe_load(f)
    pos = np.array(data["tcp_pose"]["position"], dtype=float)
    ori = np.array(data["tcp_pose"]["orientation"], dtype=float)
    return pos, ori


def load_intrinsics(intrinsics_path: Path) -> Tuple[np.ndarray, np.ndarray]:
    with open(intrinsics_path, "r") as f:
        data = json.load(f)
    camera_matrix = np.array(data["camera_matrix"], dtype=float)
    dist_coeffs = np.array(data["dist_coeffs"], dtype=float)
    return camera_matrix, dist_coeffs


def pose_to_matrix(position: np.ndarray, orientation_quat: np.ndarray) -> np.ndarray:
    rot = R.from_quat(orientation_quat).as_matrix()
    T = np.eye(4)
    T[:3, :3] = rot
    T[:3, 3] = position
    return T


def calibrate_extrinsics(
    data_path: Path,
    squares_x: int,
    squares_y: int,
    square_size: float,
    marker_size: float,
    aruco_dict: cv2.aruco_Dictionary,
) -> None:
    board = cv2.aruco.CharucoBoard(
        (squares_x, squares_y),
        square_size,
        marker_size,
        aruco_dict,
    )
    board.setLegacyPattern(True)

    image_paths = natsorted(data_path.glob("*_image_*.png"))
    tcp_paths = natsorted(data_path.glob("*_tcp_*.yaml"))
    intrinsics_path = data_path / f"{data_path.name}_intrinsics.json"

    if not image_paths:
        print(f"No images found in {data_path}")
        return
    if not tcp_paths:
        print(f"No TCP poses found in {data_path}")
        return
    if len(image_paths) != len(tcp_paths):
        print(
            f"Number of images ({len(image_paths)}) and TCP poses ({len(tcp_paths)}) do not match!"
        )
        print("Check the dataset before running extrinsic calibration")
        return
    if not intrinsics_path.is_file():
        print(f"Intrinsics file not found in {intrinsics_path}")
        return

    camera_matrix, dist_coeffs = load_intrinsics(intrinsics_path)

    tcp_poses_R = []
    tcp_poses_t = []
    target_poses_R = []
    target_poses_t = []

    valid_views = 0

    for image_path, tcp_path in zip(image_paths, tcp_paths):
        image_idx = int(image_path.stem.split("_")[-1])
        tcp_idx = int(tcp_path.stem.split("_")[-1])
        if image_idx != tcp_idx:
            print(f"Index mismatch: {image_path} vs {tcp_path}\nSkipping")
            continue

        img = cv2.imread(str(image_path))
        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = cv2.aruco.detectMarkers(img_gray, aruco_dict)
        if ids is None or len(ids) == 0:
            print(f"No ArUco markers detected in {image_path}\nSkipping")
            continue

        retval, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
            corners, ids, img_gray, board
        )
        if retval < 4:
            print(f"Not enough Charuco corners detected in view {image_path}\nSkipping")
            continue

        valid, rvec, tvec = cv2.aruco.estimatePoseBoard(
            charuco_corners, charuco_ids, board, camera_matrix, dist_coeffs, None, None
        )
        if not valid:
            print(f"Could not estimate pose for {image_path}\nSkipping")
            continue

        tcp_pos, tcp_ori = load_tcp(tcp_path)

        R_robot2tcp = R.from_quat(tcp_ori).as_matrix()
        t_robot2tcp = tcp_pos.reshape(3, 1)

        R_cam2target, _ = cv2.Rodrigues(rvec)
        t_cam2target = tvec.reshape(3, 1)

        tcp_poses_R.append(R_robot2tcp)
        tcp_poses_t.append(t_robot2tcp)

        target_poses_R.append(R_cam2target)
        target_poses_t.append(t_cam2target)

        valid_views += 1

    print(f"Processed {valid_views} valid views for extrinsic calibration")

    if valid_views < 3:
        print("Not enough valid views for calibration")
        return

    R_tcp2cam, t_tcp2cam = cv2.calibrateHandEye(
        tcp_poses_R,
        tcp_poses_t,
        target_poses_R,
        target_poses_t,
        method=cv2.CALIB_HAND_EYE_TSAI,
    )

    T_tcp2cam = np.eye(4)
    T_tcp2cam[:3, :3] = R_tcp2cam
    T_tcp2cam[:3, 3] = t_tcp2cam.ravel()

    print("Transformation matrix (TCP to camera)\n")
    print(T_tcp2cam)

    extrinsics_path = data_path / f"{data_path.name}_extrinsics.json"
    with open(extrinsics_path, "w") as f:
        json.dump({"T_tcp2cam": T_tcp2cam.tolist()}, f, indent=2)

    print(f"Extrinsics saved to {extrinsics_path}")


def main() -> None:
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

    if args.path:
        data_path = Path(args.path).expanduser() / args.camera
    else:
        data_path = Path("~/ceai_ws/calibration_data").expanduser() / args.camera

    board_path = Path(__file__).parent.parent / "config" / "charuco_board.yaml"
    with open(board_path, "r") as f:
        board_cfg = yaml.safe_load(f)

    if args.camera.startswith("tool"):
        board_params = board_cfg["tool"]
    else:
        board_params = board_cfg["scene"]

    squares_x = board_params["squares_x"]
    squares_y = board_params["squares_y"]
    square_size = board_params["square_size"]
    marker_size = board_params["marker_size"]
    aruco_dict = cv2.aruco.getPredefinedDictionary(
        getattr(cv2.aruco, board_params["aruco_dict"])
    )

    calibrate_extrinsics(
        data_path, squares_x, squares_y, square_size, marker_size, aruco_dict
    )


if __name__ == "__main__":
    main()
