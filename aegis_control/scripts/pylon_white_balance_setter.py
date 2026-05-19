#!/usr/bin/env python3
import yaml
from pathlib import Path
from enum import IntEnum

import rclpy
from rclpy.client import Client
from rclpy.node import Node
from pylon_ros2_camera_interfaces.srv import SetIntegerValue


class WhiteBalanceMode(IntEnum):
    OFF = 0
    ONCE = 1
    CONTINUOUS = 2


class PylonWhiteBalanceSetter(Node):
    def __init__(self):
        super().__init__("pylon_white_balance_setter")
        self.declare_parameter("config_file", "config/cameras/pylon_cameras.yaml")
        self.declare_parameter("camera_names", "cam_tool_left,cam_tool_right")

        config_file = Path(self.get_parameter("config_file").value)
        camera_names: str = self.get_parameter("camera_names").value

        self.camera_names: list[str] = (
            [name.strip() for name in camera_names.split(",")]
            if camera_names
            else ["cam_tool_left", "cam_tool_right"]
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
        self.set_all_white_balance()
        rclpy.shutdown()

    def create_clients(self) -> None:
        """Create service clients for all cameras"""
        self.wb_clients: dict[
            str, Client[SetIntegerValue, SetIntegerValue.Response]
        ] = {}

        for cam_name in self.camera_names:
            if cam_name in self.config:
                wb_service = f"{cam_name}/set_white_balance_auto"
                self.wb_clients[cam_name] = self.create_client(
                    SetIntegerValue, wb_service
                )
                self.get_logger().info(
                    f"Created 'set_white_balance_auto' client for {cam_name}"
                )

    def wait_for_service(self, client, timeout: float = 1.0) -> bool:
        """Wait for service with timeout"""
        while not client.wait_for_service(timeout_sec=timeout) and rclpy.ok():
            self.get_logger().info(f"Waiting for service {client.srv_name}...")
        return client.service_is_ready()

    def set_all_white_balance(self) -> None:
        """Set white balance for all configured cameras"""
        for cam_name in self.camera_names:
            if cam_name in self.config:
                wb_value = self.config[cam_name].get(
                    "white_balance_auto", WhiteBalanceMode.CONTINUOUS
                )
                self.get_logger().info(
                    f"Setting white balance for {cam_name}: {WhiteBalanceMode(wb_value).name} ({wb_value})"
                )
                self.set_white_balance(cam_name=cam_name, value=wb_value)
            else:
                self.get_logger().warning(
                    f"Missing white balance configuration for {cam_name}! Skipping."
                )

    def set_white_balance(self, cam_name: str, value: int) -> None:
        """Call set_white_balance_auto service"""
        wb_client = self.wb_clients.get(cam_name)

        if not wb_client or not self.wait_for_service(wb_client):
            self.get_logger().error(
                f"set_white_balance_auto service not available for {cam_name}"
            )
            return

        req = SetIntegerValue.Request()
        req.value = int(value)

        future = wb_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f"Successfully set white balance for {cam_name}")
        else:
            self.get_logger().error(f"Failed to set white balance for {cam_name}")


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    PylonWhiteBalanceSetter()


if __name__ == "__main__":
    main()
