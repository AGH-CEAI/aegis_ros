#!/usr/bin/env python3
"""
Sets the UR payload from aegis_description every time the robot program starts.

Workaround: on program start the robot overwrites the payload with 1.68 kg ("Custom payload"),
presumably a URCap preamble or a Set node in the pendant program. Until the cell sends its own robot
program, this re-applies the measured value on every rising edge of `robot_program_running`.
"""

from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool
from ur_msgs.srv import SetPayload


class URPayloadSetter(Node):
    def __init__(self):
        super().__init__("ur_payload_setter")
        self.declare_parameter("config_file", "")
        self.declare_parameter("controller", "io_and_status_controller")

        config_file = Path(self.get_parameter("config_file").value)
        controller = self.get_parameter("controller").value
        with open(config_file) as f:
            config = yaml.safe_load(f)
        self._mass = float(config["mass"])
        self._cog = [float(v) for v in config["center_of_gravity"]]
        self.get_logger().info(
            f"Payload from {config_file}: {self._mass} kg, CoG {self._cog} m"
        )

        self._client = self.create_client(SetPayload, f"{controller}/set_payload")
        self._program_running = False
        self.create_subscription(
            Bool,
            f"{controller}/robot_program_running",
            self._on_program_running,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )

    def _on_program_running(self, msg: Bool) -> None:
        started = msg.data and not self._program_running
        self._program_running = msg.data
        if started:
            self._set_payload()

    def _set_payload(self) -> None:
        if not self._client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f"{self._client.srv_name} is not available")
            return
        request = SetPayload.Request()
        request.mass = self._mass
        request.center_of_gravity = Vector3(
            x=self._cog[0], y=self._cog[1], z=self._cog[2]
        )
        self._client.call_async(request).add_done_callback(self._on_response)

    def _on_response(self, future) -> None:
        response = future.result()
        if response is not None and response.success:
            self.get_logger().info(f"UR payload set to {self._mass} kg")
        else:
            self.get_logger().error("Setting the UR payload failed")


def main() -> None:
    rclpy.init()
    rclpy.spin(URPayloadSetter())


if __name__ == "__main__":
    main()
