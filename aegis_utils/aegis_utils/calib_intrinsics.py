import argparse
import glob
import json
from pathlib import Path

import cv2
import yaml

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

    intrinsics_path = data_path / f"{data_path.name}_intrinsics.json"
    with open(intrinsics_path, "w") as f:
        json.dump(calib_data, f, indent=4)

    print(f"Intrinsics saved to {intrinsics_path}")


def main() -> None:
    args = parse_args()

    if args.path:
        data_path = Path(args.path).expanduser() / args.camera
        if not data_path.exists():
            print("No calibration data folder found")
            return
    else:
        data_paths = sorted(
            glob.glob(str(Path("~/ceai_ws").expanduser() / "calib_data_*")), 
            reverse=True
        )
        if data_paths:
            data_path = Path(data_paths[0]) / args.camera
        else:
            print("No calibration data folder found")
            return

    board_path = Path(__file__).parent.parent / "config" / "charuco_board.yaml"
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
