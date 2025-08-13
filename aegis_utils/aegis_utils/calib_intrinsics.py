import argparse
import glob
import json
import os

import cv2
from natsort import natsorted

CAMERA_CONFIG = {
    "tool_front_right": {},
    "tool_front_left": {},
    "tool_right": {},
    "tool_left": {},
    "scene": {},
}

def calibrate_intrinsics(data_path: str) -> None:
    squares_x = 9
    squares_y = 6
    square_size = 0.03
    marker_size = 0.022

    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_1000)
    board = cv2.aruco.CharucoBoard((squares_x, squares_y), square_size, marker_size, aruco_dict)
    board.setLegacyPattern(True)

    images = natsorted(glob.glob(os.path.join(os.path.expanduser(data_path), "*.png")))

    if len(images) == 0:
        print(f"No images found in {data_path}")
        return

    all_corners = []
    all_ids = []
    image_size = None

    for fname in images:
        img = cv2.imread(fname)
        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        image_size = img_gray.shape[::-1]

        corners, ids, _ = cv2.aruco.detectMarkers(img_gray, aruco_dict)
        if ids is not None and len(ids) > 0:
            retval, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(corners, ids, img_gray, board)
            print(f"{retval} ChArUco corners were detected in image {os.path.basename(fname)}")
            if charuco_ids is not None and len(charuco_ids) > 3:
                all_corners.append(charuco_corners)
                all_ids.append(charuco_ids)

    print(f"Found {len(all_corners)} valid images for calibration")

    if len(all_corners) == 0:
        print("No valid data for calibration")
        return

    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(
        charucoCorners=all_corners,
        charucoIds=all_ids,
        board=board,
        imageSize=image_size,
        cameraMatrix=None,
        distCoeffs=None
    )

    print("Camera matrix (K):\n", camera_matrix)
    print("Distortion coefficients (D):\n", dist_coeffs)

    calib_data = {
        "camera_matrix": camera_matrix.tolist(),
        "dist_coeffs": dist_coeffs.tolist(),
        "square_size": square_size,
        "marker_size": marker_size,
        "squares_x": squares_x,
        "squares_y": squares_y
    }

    intrinsics_path = os.path.join(data_path, f"{os.path.basename(data_path)}_camera_intrinsics.json")
    with open(intrinsics_path, "w") as f:
        json.dump(calib_data, f, indent=4)

    print(f"Intrinsics saved to {intrinsics_path}")


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
        data_path = os.path.join(os.path.expanduser(args.path), args.camera)
    else:
        data_path = os.path.expanduser(f"~/ceai_ws/calibration_data/{args.camera}")

    calibrate_intrinsics(data_path)


if __name__ == "__main__":
    main()
