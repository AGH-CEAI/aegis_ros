import argparse
import glob
import json
from pathlib import Path
from typing import Dict, List, NamedTuple, Optional, Tuple

import cv2
import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
from scipy.spatial.transform import Rotation

CAMERA_CONFIG = {
    "scene": {},
    "tool_front_right": {},
    "tool_front_left": {},
    "tool_right": {},
    "tool_left": {},
}


class ViewTransforms(NamedTuple):
    R_base2tcp: np.ndarray
    t_base2tcp: np.ndarray
    R_tcp2base: np.ndarray
    t_tcp2base: np.ndarray
    R_cam2target: np.ndarray
    t_cam2target: np.ndarray


def main() -> None:
    args = parse_args()
    cam_name = args.camera
    path_name = args.path

    base_path = (
        Path(path_name).expanduser()
        if args.path
        else Path("~/ceai_ws/calib_data").expanduser()
    )
    data_path = get_latest_folder(base_path, cam_name)
    if not data_path:
        print(f"No calibration data folder found in {base_path} directory")
        return

    package_share_path = Path(get_package_share_directory("aegis_utils"))
    board_path = package_share_path / "config" / "charuco_board.yaml"

    with open(board_path, "r") as f:
        board_cfg = yaml.safe_load(f)

    if cam_name.startswith("tool_front"):
        board_params = board_cfg["big"]
    else:
        board_params = board_cfg["small"]

    squares_x = board_params["squares_x"]
    squares_y = board_params["squares_y"]
    square_size = board_params["square_size"]
    marker_size = board_params["marker_size"]
    aruco_dict = cv2.aruco.getPredefinedDictionary(
        getattr(cv2.aruco, board_params["aruco_dict"])
    )

    calibrate_extrinsics(
        cam_name, data_path, squares_x, squares_y, square_size, marker_size, aruco_dict
    )


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


def get_latest_folder(base_path: Path, camera: str) -> Optional[Path]:
    folders = glob.glob(str(base_path / f"{camera}_*"))
    if not folders:
        return None
    latest_folder = sorted(folders, reverse=True)[0]
    return Path(latest_folder)


def calibrate_extrinsics(
    cam_name: str,
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

    image_paths = sorted(data_path.glob("*_image_*.png"))
    tcp_paths = sorted(data_path.glob("*_tcp_*.yaml"))
    intrinsics_path = data_path / f"{cam_name}_intrinsics.json"
    extrinsics_path = data_path / f"{cam_name}_extrinsics.json"

    if not image_paths:
        print(f"No images found in {data_path}")
        return
    if not tcp_paths:
        print(f"No TCP poses found in {data_path}")
        return
    if not intrinsics_path.is_file():
        print(f"Intrinsics file not found in {intrinsics_path}")
        return
    if len(image_paths) != len(tcp_paths):
        print(
            f"Number of images ({len(image_paths)}) and TCP poses ({len(tcp_paths)}) do not match!"
        )

    camera_matrix, dist_coeffs = load_intrinsics(intrinsics_path)

    base2tcp_list_R = []
    base2tcp_list_t = []
    tcp2base_list_R = []
    tcp2base_list_t = []
    cam2target_list_R = []
    cam2target_list_t = []

    valid_views = 0

    for image_path, tcp_path in zip(image_paths, tcp_paths):
        transforms = process_view(
            image_path, tcp_path, board, aruco_dict, camera_matrix, dist_coeffs
        )
        if transforms is None:
            continue

        base2tcp_list_R.append(transforms.R_base2tcp)
        base2tcp_list_t.append(transforms.t_base2tcp)
        tcp2base_list_R.append(transforms.R_tcp2base)
        tcp2base_list_t.append(transforms.t_tcp2base)
        cam2target_list_R.append(transforms.R_cam2target)
        cam2target_list_t.append(transforms.t_cam2target)

        valid_views += 1

    print(f"Processed {valid_views} valid views for extrinsic calibration")

    if valid_views < 3:
        print("Not enough valid views for calibration")
        return

    if cam_name == "scene":
        calib_data = calibrate_eye_to_hand(
            tcp2base_list_R, tcp2base_list_t, cam2target_list_R, cam2target_list_t
        )
    else:
        calib_data = calibrate_eye_in_hand(
            base2tcp_list_R, base2tcp_list_t, cam2target_list_R, cam2target_list_t
        )

    with open(extrinsics_path, "w") as f:
        json.dump(calib_data, f, indent=2)
        f.write("\n")

    print(f"Extrinsics saved to {extrinsics_path}")


def load_intrinsics(intrinsics_path: Path) -> Tuple[np.ndarray, np.ndarray]:
    with open(intrinsics_path, "r") as f:
        data = json.load(f)
    camera_matrix = np.array(data["camera_matrix"], dtype=float)
    dist_coeffs = np.array(data["dist_coeffs"], dtype=float)
    return camera_matrix, dist_coeffs


def process_view(
    image_path: Path,
    tcp_path: Path,
    board: cv2.aruco_CharucoBoard,
    aruco_dict: cv2.aruco_Dictionary,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
) -> Optional[ViewTransforms]:
    try:
        image_idx = int(image_path.stem.split("_")[-1])
        tcp_idx = int(tcp_path.stem.split("_")[-1])
    except ValueError:
        print(f"Invalid index in filenames: {image_path}, {tcp_path}\nSkipping")
        return None

    if image_idx != tcp_idx:
        print(f"Index mismatch: {image_path} vs {tcp_path}\nSkipping")
        return None

    img = cv2.imread(str(image_path))
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = cv2.aruco.detectMarkers(img_gray, aruco_dict)

    if ids is None or len(ids) == 0:
        print(f"No ArUco markers detected in {image_path}\nSkipping")
        return None

    retval, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
        corners, ids, img_gray, board
    )

    if retval < 4:
        print(f"Not enough Charuco corners detected in view {image_path}\nSkipping")
        return None

    valid, rvec, tvec = cv2.aruco.estimatePoseBoard(
        charuco_corners, charuco_ids, board, camera_matrix, dist_coeffs, None, None
    )

    if not valid:
        print(f"Could not estimate pose for {image_path}\nSkipping")
        return None

    tcp_pos, tcp_ori = load_tcp(tcp_path)

    R_base2tcp = Rotation.from_quat(tcp_ori).as_matrix()
    t_base2tcp = tcp_pos.reshape(3, 1)

    R_tcp2base = R_base2tcp.T
    t_tcp2base = -R_base2tcp.T @ t_base2tcp

    R_cam2target, _ = cv2.Rodrigues(rvec)
    t_cam2target = tvec.reshape(3, 1)

    return ViewTransforms(
        R_base2tcp, t_base2tcp, R_tcp2base, t_tcp2base, R_cam2target, t_cam2target
    )


def load_tcp(tcp_path: Path) -> Tuple[np.ndarray, np.ndarray]:
    with open(tcp_path, "r") as f:
        data = yaml.safe_load(f)
    pos = np.array(data["tcp_pose"]["position"], dtype=float)
    ori = np.array(data["tcp_pose"]["orientation"], dtype=float)
    return pos, ori


def calibrate_eye_to_hand(
    tcp2base_list_R: List[np.ndarray],
    tcp2base_list_t: List[np.ndarray],
    cam2target_list_R: List[np.ndarray],
    cam2target_list_t: List[np.ndarray],
) -> Dict[str, List[List[float]]]:
    R_base2cam, t_base2cam = cv2.calibrateHandEye(
        tcp2base_list_R,
        tcp2base_list_t,
        cam2target_list_R,
        cam2target_list_t,
        method=cv2.CALIB_HAND_EYE_TSAI,
    )

    T_base2cam = make_homogeneous(R_base2cam, t_base2cam)
    T_cam2base = np.linalg.inv(T_base2cam)

    print("Transformation matrix (base to camera):\n", T_base2cam)
    print("Transformation matrix (camera to base):\n", T_cam2base)

    calib_data = {
        "T_base2cam": T_base2cam.tolist(),
        "T_cam2base": T_cam2base.tolist(),
    }

    return calib_data


def calibrate_eye_in_hand(
    base2tcp_list_R: List[np.ndarray],
    base2tcp_list_t: List[np.ndarray],
    cam2target_list_R: List[np.ndarray],
    cam2target_list_t: List[np.ndarray],
) -> Dict[str, List[List[float]]]:
    R_tcp2cam, t_tcp2cam = cv2.calibrateHandEye(
        base2tcp_list_R,
        base2tcp_list_t,
        cam2target_list_R,
        cam2target_list_t,
        method=cv2.CALIB_HAND_EYE_TSAI,
    )

    T_tcp2cam = make_homogeneous(R_tcp2cam, t_tcp2cam)
    T_cam2tcp = np.linalg.inv(T_tcp2cam)

    print("Transformation matrix (TCP to camera):\n", T_tcp2cam)
    print("Transformation matrix (camera to TCP):\n", T_cam2tcp)

    calib_data = {
        "T_tcp2cam": T_tcp2cam.tolist(),
        "T_cam2tcp": T_cam2tcp.tolist(),
    }

    return calib_data


def make_homogeneous(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t.ravel()
    return T


if __name__ == "__main__":
    main()
