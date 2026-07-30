#!/usr/bin/env python3
from pathlib import Path

import rclpy
import yaml
from pylon_ros2_camera_interfaces.srv import SetROI
from rclpy.client import Client
from rclpy.node import Node


class PylonROISetter(Node):
    def __init__(self):
        super().__init__("pylon_roi_setter")
        self.declare_parameter("config_file", "config/cameras/roi.yaml")
        self.declare_parameter("camera_names", "cam_tool_left,cam_tool_right")

        config_file = Path(self.get_parameter("config_file").value)
        camera_names: str = self.get_parameter("camera_names").value

        self.camera_names: list[str] = (
            [name.strip() for name in camera_names.split(",")]
            if camera_names
            else ["basler_left", "basler_right"]
        )

        # Load config from YAML
        try:
            with open(config_file, "r") as f:
                self.config: dict[str, dict[str, int]] = yaml.safe_load(f)
            self.get_logger().info(f"Loaded config from {config_file}")
        except Exception as e:
            self.get_logger().error(f"Failed to load {config_file}: {e}")
            return

        self.create_clients()
        self.set_all_rois()
        rclpy.shutdown()

    def create_clients(self) -> None:
        """Create service clients for all cameras"""
        self.set_roi_clients: dict[str, Client[SetROI, SetROI.Response]] = {}

        for cam_name in self.camera_names:
            if cam_name in self.config:
                roi_service = f"{cam_name}/set_roi"
                self.set_roi_clients[cam_name] = self.create_client(SetROI, roi_service)

                self.get_logger().info(f"Created 'set_roi' client for {cam_name}")

    def wait_for_service(self, client, timeout: float = 1.0) -> bool:
        """Wait for service with timeout"""
        while not client.wait_for_service(timeout_sec=timeout) and rclpy.ok():
            self.get_logger().info(f"Waiting for service {client.srv_name}...")
        return client.service_is_ready()

    def set_all_rois(self) -> None:
        """Set ROI for all configured cameras"""
        for cam_name in self.camera_names:
            if cam_name in self.config:
                self.get_logger().info(
                    f"Setting ROI for {cam_name}: {self.config[cam_name]}"
                )
                self.set_roi(cam_name=cam_name)
            else:
                self.get_logger().warning(
                    f"Missing ROI configuration for {cam_name}! Skipping ROI setup"
                )

    def set_roi(self, cam_name: str) -> None:
        """Set ROI using set_roi service"""
        cfg = self.config[cam_name]
        roi_client = self.set_roi_clients[cam_name]

        if not roi_client or not self.wait_for_service(roi_client):
            self.get_logger().error(f"set_roi service not available for {cam_name}")
            return

        req = SetROI.Request()
        req.target_roi.width = int(cfg["roi_width"])
        req.target_roi.height = int(cfg["roi_height"])
        req.target_roi.x_offset = int(cfg["offset_x"])
        req.target_roi.y_offset = int(cfg["offset_y"])
        req.target_roi.do_rectify = (
            False  # This should be False if the full image is captured (ROI not used)
        )

        future = roi_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f"Successfully set ROI for {cam_name}")
        else:
            self.get_logger().error(f"Failed to set ROI for {cam_name}")


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    PylonROISetter()


if __name__ == "__main__":
    main()
