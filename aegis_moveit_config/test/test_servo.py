import rclpy
from controller_manager_msgs.srv import SwitchController
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from std_srvs.srv import Trigger


class ServoController:
    """Helper class to manage servo enable/disable with automatic state detection."""

    def __init__(self, node: Node):
        self.node = node
        self.servo_enabled = False

        # Service clients
        self.start_servo_client = node.create_client(Trigger, "/servo_node/start_servo")
        self.stop_servo_client = node.create_client(Trigger, "/servo_node/stop_servo")
        self.switch_controller_client = node.create_client(
            SwitchController, "/controller_manager/switch_controller"
        )

        # Wait for services (FIXED: use timeout loop, not spin_until_future_complete)
        self.node.get_logger().info("Waiting for servo control services...")

        # Wait for start_servo
        while rclpy.ok() and not self.start_servo_client.wait_for_service(
            timeout_sec=1.0
        ):
            self.node.get_logger().info("Waiting for /servo_node/start_servo...")

        # Wait for stop_servo
        while rclpy.ok() and not self.stop_servo_client.wait_for_service(
            timeout_sec=1.0
        ):
            self.node.get_logger().info("Waiting for /servo_node/stop_servo...")

        # Wait for switch_controller
        while rclpy.ok() and not self.switch_controller_client.wait_for_service(
            timeout_sec=1.0
        ):
            self.node.get_logger().info(
                "Waiting for /controller_manager/switch_controller..."
            )

        self.node.get_logger().info("All servo control services ready!")

    def check_servo_state(self) -> bool:
        """Check if servo is currently enabled by calling start_servo service."""
        if not self.start_servo_client.wait_for_service(timeout_sec=1.0):
            return False

        request = Trigger.Request()
        future = self.start_servo_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)

        if future.result() is not None and future.result().success:
            # If start_servo succeeded, servo was already enabled
            self.servo_enabled = True
            self.node.get_logger().info("Servo already enabled")
            return True
        else:
            self.servo_enabled = False
            return False

    def enable_servo(self) -> bool:
        """Enable servo mode: switch controllers + start servo."""
        self.node.get_logger().info("Enabling servo mode...")

        # Switch controllers: deactivate trajectory, activate forward_position
        switch_request = SwitchController.Request()
        switch_request.start_controllers = ["forward_position_controller"]
        switch_request.stop_controllers = ["scaled_joint_trajectory_controller"]
        switch_request.strictness = 2  # BEST_EFFORT

        future = self.switch_controller_client.call_async(switch_request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)

        if future.result() is None or not future.result().success:
            self.node.get_logger().error("Failed to switch controllers")
            return False

        self.node.get_logger().info("Controllers switched successfully")

        # Start servo service
        request = Trigger.Request()
        future = self.start_servo_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)

        if future.result() is not None and future.result().success:
            self.servo_enabled = True
            self.node.get_logger().info("Servo started successfully")
            return True
        else:
            self.node.get_logger().error("Failed to start servo")
            return False

    def disable_servo(self) -> bool:
        """Disable servo mode: stop servo + switch controllers back."""
        if not self.servo_enabled:
            self.node.get_logger().info("Servo already disabled")
            return True

        self.node.get_logger().info("Disabling servo mode...")

        # Stop servo service first
        request = Trigger.Request()
        future = self.stop_servo_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)

        if future.result() is None or not future.result().success:
            self.node.get_logger().error("Failed to stop servo")
            return False

        # Switch controllers back: activate trajectory, deactivate forward_position
        switch_request = SwitchController.Request()
        switch_request.start_controllers = ["scaled_joint_trajectory_controller"]
        switch_request.stop_controllers = ["forward_position_controller"]
        switch_request.strictness = 2  # BEST_EFFORT

        future = self.switch_controller_client.call_async(switch_request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)

        if future.result() is None or not future.result().success:
            self.node.get_logger().error("Failed to switch controllers back")
            return False

        self.servo_enabled = False
        self.node.get_logger().info("Servo mode disabled successfully")
        return True


class ServoTwistPublisher(Node):
    def __init__(self):
        super().__init__("servo_twist_publisher")

        # Parameters
        self.declare_parameter("frequency_msg_twist", 250.0)
        self.declare_parameter("msgs", 250)
        self.declare_parameter("x", 1.0)
        self.declare_parameter("y", 0.0)
        self.declare_parameter("z", 0.0)
        self.declare_parameter("rx", 0.0)
        self.declare_parameter("ry", 0.0)
        self.declare_parameter("rz", 0.0)

        self.rate = self.get_parameter("frequency_msg_twist").value
        self.num_messages = self.get_parameter("msgs").value
        self.linear_x = float(self.get_parameter("x").value)
        self.linear_y = float(self.get_parameter("y").value)
        self.linear_z = float(self.get_parameter("z").value)
        self.angular_x = float(self.get_parameter("rx").value)
        self.angular_y = float(self.get_parameter("ry").value)
        self.angular_z = float(self.get_parameter("rz").value)

        # Publisher
        self.publisher = self.create_publisher(
            TwistStamped, "/servo_node/delta_twist_cmds", 10
        )

        # Servo controller
        # self.servo_ctrl = ServoController(self)

        # Timer
        timer_period = 1.0 / self.rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.msg_count = 0
        self.servo_was_already_enabled = False

        self.get_logger().info(
            f"Ready to publish {self.num_messages} messages at {self.rate} Hz"
        )

    def timer_callback(self):
        if self.msg_count < self.num_messages:
            # Publish servo twist command
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "base_link"
            msg.twist.linear.x = self.linear_x
            msg.twist.linear.y = self.linear_y
            msg.twist.linear.z = self.linear_z
            msg.twist.angular.x = self.angular_x
            msg.twist.angular.y = self.angular_y
            msg.twist.angular.z = self.angular_z
            self.publisher.publish(msg)
            self.msg_count += 1
            self.get_logger().info(
                f"Published msg {self.msg_count}/{self.num_messages}"
            )

        elif self.msg_count >= self.num_messages:
            # Send final zero command
            zero_msg = TwistStamped()
            zero_msg.header.stamp = self.get_clock().now().to_msg()
            zero_msg.header.frame_id = "base_link"
            self.publisher.publish(zero_msg)

            # Stop timer
            self.timer.cancel()

            # # Disable servo (unless it was already enabled before we started)
            # if not self.servo_was_already_enabled:
            #     self.servo_ctrl.disable_servo()

            self.get_logger().info("Transmission complete!")
            exit()


def main(args=None):
    rclpy.init(args=args)

    node = ServoTwistPublisher()

    # Check servo state and enable if needed
    # if not node.servo_ctrl.check_servo_state():
    #     success = node.servo_ctrl.enable_servo()
    #     if not success:
    #         node.get_logger().error('Failed to enable servo. Exiting.')
    #         return
    # else:
    #     node.servo_was_already_enabled = True
    #     node.get_logger().info('Servo was already enabled')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user")
        try:
            if not node.servo_was_already_enabled:
                node.servo_ctrl.disable_servo()
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
