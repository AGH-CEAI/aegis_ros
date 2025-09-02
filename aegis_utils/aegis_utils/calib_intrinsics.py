import argparse
import glob
import json
from pathlib import Path
from typing import Optional

import cv2
import yaml
from ament_index_python.packages import get_package_share_directory

CAMERA_CONFIG = {
    "scene": {},
    "tool_front_right": {},
    "tool_front_left": {},
    "tool_right": {},
    "tool_left": {},
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


def get_latest_folder(base_path: Path, camera: str) -> Optional[Path]:
    folders = glob.glob(str(base_path / f"{camera}_*"))
    if not folders:
        return None
    latest_folder = sorted(folders, reverse=True)[0]
    return Path(latest_folder)


def calibrate_intrinsics(
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

    cam_name = data_path.name[:-18]
    image_paths = sorted(data_path.glob("*_image_*.png"))

    if not image_paths:
        print(f"No images found in {data_path}")
        return

    all_corners = []
    all_ids = []
    image_size = None

    for image_path in image_paths:
        img = cv2.imread(str(image_path))
        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        image_size = img_gray.shape[::-1]

        corners, ids, _ = cv2.aruco.detectMarkers(img_gray, aruco_dict)
        if ids is not None and len(ids) > 0:
            retval, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
                corners, ids, img_gray, board
            )
            print(f"{retval} ChArUco corners were detected in view {image_path.name}")
            if charuco_ids is not None and len(charuco_ids) > 3:
                all_corners.append(charuco_corners)
                all_ids.append(charuco_ids)

    print(f"Found {len(all_corners)} valid views for intrinsic calibration")

    if not all_corners:
        print("No valid views for calibration")
        return

    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(
        charucoCorners=all_corners,
        charucoIds=all_ids,
        board=board,
        imageSize=image_size,
        cameraMatrix=None,
        distCoeffs=None,
    )

    print("Camera matrix (K):\n", camera_matrix)
    print("Distortion coefficients (D):\n", dist_coeffs)

    calib_data = {
        "camera_matrix": camera_matrix.tolist(),
        "dist_coeffs": dist_coeffs.tolist(),
        "square_size": square_size,
        "marker_size": marker_size,
        "squares_x": squares_x,
        "squares_y": squares_y,
    }

    intrinsics_path = data_path / f"{cam_name}_intrinsics.json"
    with open(intrinsics_path, "w") as f:
        json.dump(calib_data, f, indent=4)

    print(f"Intrinsics saved to {intrinsics_path}")


def main() -> None:
    args = parse_args()

    base_path = (
        Path(args.path).expanduser()
        if args.path
        else Path("~/ceai_ws/calib_data").expanduser()
    )
    data_path = get_latest_folder(base_path, args.camera)
    if not data_path:
        print(f"No calibration data folder found in {base_path} directory")
        return

    package_share_path = Path(get_package_share_directory("aegis_utils"))
    board_path = package_share_path / "config" / "charuco_board.yaml"

    with open(board_path, "r") as f:
        board_cfg = yaml.safe_load(f)

    if args.camera.startswith("tool_front"):
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

    calibrate_intrinsics(
        data_path, squares_x, squares_y, square_size, marker_size, aruco_dict
    )


if __name__ == "__main__":
    main()
