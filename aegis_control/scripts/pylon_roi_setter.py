#!/usr/bin/env python3
import sys
import yaml
import rclpy
from rclpy.node import Node
from pylon_ros2_camera_interfaces.srv import SetROI, SetInteger


class PylonROISetter(Node):
    def __init__(
        self, config_file, camera_names, left_service_prefix, right_service_prefix
    ):
        super().__init__("pylon_roi_setter")

        self.camera_names = (
            camera_names.split(",") if camera_names else "basler_left,basler_right"
        )
        self.left_service_prefix = left_service_prefix
        self.right_service_prefix = right_service_prefix

        # Load config from YAML
        try:
            with open(config_file, "r") as f:
                self.config = yaml.safe_load(f)
            self.get_logger().info(f"Loaded config from {config_file}")
        except Exception as e:
            self.get_logger().error(f"Failed to load {config_file}: {e}")
            return

        # Create service clients
        self.create_clients()

        # Set ROIs for all cameras
        self.set_all_rois()

        # Exit after completion
        rclpy.shutdown()

    def create_clients(self):
        """Create service clients for all cameras"""
        self.set_roi_clients = {}
        self.set_offset_x_clients = {}
        self.set_offset_y_clients = {}

        for cam_name in self.camera_names.split(","):
            cam_name = cam_name.strip()
            if cam_name in self.config:
                prefix = (
                    self.left_service_prefix
                    if "left" in cam_name.lower()
                    else self.right_service_prefix
                )

                # SetROI service
                roi_service = f"{prefix}/{cam_name}/set_roi"
                self.set_roi_clients[cam_name] = self.create_client(SetROI, roi_service)

                # Individual offset services (if available)
                x_service = f"{prefix}/{cam_name}/set_offset_x"
                y_service = f"{prefix}/{cam_name}/set_offset_y"
                self.set_offset_x_clients[cam_name] = self.create_client(
                    SetInteger, x_service
                )
                self.set_offset_y_clients[cam_name] = self.create_client(
                    SetInteger, y_service
                )

                self.get_logger().info(f"Created clients for {cam_name}: {roi_service}")

    def wait_for_service(self, client, timeout=3.0):
        """Wait for service with timeout"""
        while not client.wait_for_service(timeout_sec=1.0) and rclpy.ok():
            self.get_logger().info(f"Waiting for service {client.srv_name}...")
        return client.service_is_ready()

    def set_all_rois(self):
        """Set ROI for all configured cameras"""
        for cam_name in self.camera_names.split(","):
            cam_name = cam_name.strip()
            if cam_name in self.config:
                config = self.config[cam_name]
                self.get_logger().info(f"Setting ROI for {cam_name}: {config}")
                self.set_roi(cam_name, config)

    def set_roi(self, cam_name, config):
        """Set ROI using SetROI service"""
        roi_client = self.set_roi_clients.get(cam_name)
        if not roi_client or not self.wait_for_service(roi_client):
            self.get_logger().error(f"SetROI service not available for {cam_name}")
            return

        req = SetROI.Request()
        req.offset_x = int(config["offset_x"])
        req.offset_y = int(config["offset_y"])
        req.width = int(config["roi_width"])
        req.height = int(config["roi_height"])

        future = roi_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f"Successfully set ROI for {cam_name}")
        else:
            self.get_logger().error(f"Failed to set ROI for {cam_name}")


def main(args=None):
    rclpy.init(args=args)

    # Default values
    config_file = "config/cameras_roi.yaml"
    camera_names = "basler_left,basler_right"
    left_service_prefix = "cam_tool_left"
    right_service_prefix = "cam_tool_right"

    # Parse command line arguments (from launch file)
    if len(sys.argv) >= 2:
        config_file = sys.argv[1]
    if len(sys.argv) >= 3:
        camera_names = sys.argv[2]
    if len(sys.argv) >= 4:
        left_service_prefix = sys.argv[3]
    if len(sys.argv) >= 5:
        right_service_prefix = sys.argv[4]

    node = PylonROISetter(
        config_file, camera_names, left_service_prefix, right_service_prefix
    )


if __name__ == "__main__":
    main()
