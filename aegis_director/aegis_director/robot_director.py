from threading import Thread

import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from sensor_msgs.msg import JointState

from pymoveit2 import MoveIt2, MoveIt2State
import aegis_director.aegis_robot as robot


class RobotDirector:
    def __init__(self, node: Node, synchronous: bool = True):
        self.synchronous = synchronous
        self.node = node
        self.callback_group = ReentrantCallbackGroup()

        self.moveit2 = MoveIt2(
            node=self.node,
            joint_names=robot.joint_names(),
            base_link_name=robot.base_link_name(),
            end_effector_name=robot.end_effector_name(),
            group_name=robot.MOVE_GROUP_ARM,
            callback_group=self.callback_group,
        )
        self.moveit2.planner_id = "RRTConnectkConfigDefault"

        self.executor = rclpy.executors.MultiThreadedExecutor(2)
        self.executor.add_node(self.node)
        self.executor_thread = Thread(target=self.executor.spin, daemon=True, args=())
        self.executor_thread.start()
        self.node.create_rate(1.0).sleep()

    def __del__(self):
        if self.executor_thread.is_alive():
            self.executor_thread.join()
        self.executor.shutdown()

    def get_joint_states(self) -> dict[str, float]:
        js = self._get_joint_states()
        return {name: position for name, position in zip(js.name, js.position)}

    def _get_joint_states(self) -> JointState:
        return self.moveit2.joint_state

    def get_tcp_pose(self) -> dict[str, np.ndarray]:
        pose = self._get_tcp_pose()
        pos = pose.position
        ori = pose.orientation
        return {
            "position": np.array([pos.x, pos.y, pos.z]),
            "orientation": np.array([ori.x, ori.y, ori.z, ori.w]),
        }

    def _get_tcp_pose(self) -> Pose:
        js = self.moveit2.joint_state
        retval = None
        if self.synchronous:
            retval = self.moveit2.compute_fk(js)
        else:
            future = self.moveit2.compute_fk_async(js)
            if future is not None:
                rate = self.node.create_rate(10)
                while not future.done():
                    rate.sleep()
                retval = self.moveit2.get_compute_fk_result(future)
        if retval is None:
            raise ValueError("Failed to obtain TCP pose (i.e. calculate FK).")
        return retval.pose

    def joint_move(
        self,
        joint_positions: dict[str, float],
        max_vel: float = 1.0,
        max_accel: float = 1.0,
        cancel_after_secs: float = 0.0,
    ) -> None:
        formatted_joints = {k: f"{v:.3f}" for k, v in joint_positions.items()}
        self.node.get_logger().info(
            f"Moving to {{joints: {formatted_joints}, max_vel: {max_vel:.2f}, max_accel: {max_accel:.2f}}}"
        )

        self.moveit2.max_velocity = max_vel
        self.moveit2.max_acceleration = max_accel
        self.moveit2.move_to_configuration(
            joint_positions=list(joint_positions.values()),
            joint_names=list(joint_positions.keys()),
            tolerance=0.001,
            weight=1.0,
        )
        self._wait_for_execution(cancel_after_secs)

    def cartesian_move(self):
        # Placeholder for future implementation
        pass

    def gripper_move(self):
        # Placeholder for future implementation
        pass

    def servo_move(self):
        # Placeholder for future implementation
        pass

    def _wait_for_execution(self, cancel_after_secs: float = 0.0) -> None:
        if self.synchronous:
            self.moveit2.wait_until_executed()
            return

        self.node.get_logger().info("Current State: " + str(self.moveit2.query_state()))
        rate = self.node.create_rate(10)
        while self.moveit2.query_state() != MoveIt2State.EXECUTING:
            rate.sleep()

        self.node.get_logger().info("Current State: " + str(self.moveit2.query_state()))
        future = self.moveit2.get_execution_future()

        if cancel_after_secs > 0.0:
            sleep_time = self.node.create_rate(cancel_after_secs)
            sleep_time.sleep()
            self.node.get_logger().info("Cancelling goal")
            self.moveit2.cancel_execution()

        while not future.done():
            rate.sleep()

        self.node.get_logger().info("Result status: " + str(future.result().status))
        self.node.get_logger().info(
            "Result error code: " + str(future.result().result.error_code)
        )
