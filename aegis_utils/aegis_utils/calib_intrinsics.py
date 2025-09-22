import argparse
import glob
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


def main() -> None:
    args = parse_args()
    cam_name = args.camera
    path_name = args.path

    base_path = (
        Path(path_name).expanduser()
        if path_name
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

    calibrate_intrinsics(
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


def calibrate_intrinsics(
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
        image_width, image_height = image_size

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
        "image_width": image_width,
        "image_height": image_height,
        "camera_name": cam_name,
        "camera_matrix": {
            "rows": 3,
            "cols": 3,
            "data": camera_matrix.flatten().tolist(),
        },
        "distortion_model": "plumb_bob",
        "distortion_coefficients": {
            "rows": 1,
            "cols": dist_coeffs.size,
            "data": dist_coeffs.flatten().tolist(),
        },
        "rectification_matrix": {
            "rows": 3,
            "cols": 3,
            "data": [1, 0, 0, 0, 1, 0, 0, 0, 1],
        },
        "projection_matrix": {
            "rows": 3,
            "cols": 4,
            "data": [
                float(camera_matrix[0, 0]),
                0,
                float(camera_matrix[0, 2]),
                0,
                0,
                float(camera_matrix[1, 1]),
                float(camera_matrix[1, 2]),
                0,
                0,
                0,
                1,
                0,
            ],
        },
    }

    intrinsics_path = data_path / f"{cam_name}_intrinsics.yaml"
    with open(intrinsics_path, "w") as f:
        yaml.safe_dump(
            calib_data,
            f,
            sort_keys=False,
            default_flow_style=None,
            width=200,
        )

    print(f"Intrinsics saved to {intrinsics_path}")


if __name__ == "__main__":
    main()
