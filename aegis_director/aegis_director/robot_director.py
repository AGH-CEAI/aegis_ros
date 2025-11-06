import time
from threading import Thread
from typing import Iterable, Optional, Union

import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from controller_manager_msgs.srv import SwitchController
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from sensor_msgs.msg import JointState

from pymoveit2 import MoveIt2, MoveIt2State, GripperInterface, MoveIt2Servo
import aegis_director.aegis_robot as robot


class RobotDirector:
    def __init__(self, synchronous: bool = True):
        self.synchronous = synchronous
        self.node = Node("director")
        self.cb_group = ReentrantCallbackGroup()
        self.cb_exclusive_group = MutuallyExclusiveCallbackGroup()

        self._prepare_moveit2()
        self._preapre_servo()
        self._preapre_gripper_interface()

        self.executor = rclpy.executors.MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self.executor_thread = Thread(target=self.executor.spin, daemon=True, args=())
        self.executor_thread.start()

        # Ensure that the servo is disabled
        self._servo_enabled = True
        self.servo_disable()

    def _prepare_moveit2(self) -> None:
        self.moveit2 = MoveIt2(
            node=self.node,
            joint_names=robot.joint_names(),
            base_link_name=robot.base_link_name(),
            end_effector_name=robot.end_effector_name(),
            group_name=robot.MOVE_GROUP_ARM,
            callback_group=self.cb_group,
        )
        self.moveit2.planner_id = "RRTConnectkConfigDefault"

    def _preapre_servo(self) -> None:
        self.servo = MoveIt2Servo(
            node=self.node,
            frame_id=robot.base_link_name(),
            callback_group=self.cb_group,
            enable_at_init=False,
        )
        self.switch_controllers = self.node.create_client(
            SwitchController,
            "/controller_manager/switch_controller",
            callback_group=self.cb_exclusive_group,
        )
        if not self.switch_controllers.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().warn(
                f"Service {self.switch_controllers.srv_name} not available"
            )

    @property
    def servo_enabled(self) -> bool:
        return self._servo_enabled

    def _preapre_gripper_interface(self) -> None:
        self.gripper_interface = GripperInterface(
            node=self.node,
            gripper_joint_names=robot.gripper_joint_names(),
            open_gripper_joint_positions=robot.OPEN_GRIPPER_JOINT_POSITIONS,
            closed_gripper_joint_positions=robot.CLOSED_GRIPPER_JOINT_POSITIONS,
            gripper_group_name=robot.MOVE_GROUP_GRIPPER,
            callback_group=self.cb_group,
            gripper_command_action_name="gripper_action_controller/gripper_cmd",
        )

    def __del__(self):
        self.executor.shutdown()
        if self.executor_thread.is_alive():
            self.executor_thread.join()
        self.node.destroy_node()

    def get_joint_positions(self) -> dict[str, float]:
        js = self._get_joint_states()
        return {name: position for name, position in zip(js.name, js.position)}

    def get_joint_velocities(self) -> dict[str, float]:
        js = self._get_joint_states()
        return {name: position for name, position in zip(js.name, js.velocity)}

    def get_joint_effort(self) -> dict[str, float]:
        js = self._get_joint_states()
        return {name: position for name, position in zip(js.name, js.effort)}

    def _get_joint_states(self) -> JointState:
        js = self.moveit2.joint_state
        while js is None:
            self.node.create_rate(10.0).sleep()
            js = self.moveit2.joint_state
        return js

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

    def servo_enable(self) -> None:
        if self.servo_enabled:
            return
        self.servo.enable(sync=False)
        self._switch_controllers(
            activate=["forward_position_controller"],
            deactivate=["scaled_joint_trajectory_controller"],
        )

        # TODO(issue#X) ROS spin deadlockswith synchronous calls
        # HACK workaround for sync wait deadlock
        time.sleep(1.0)
        self.servo.__is_enabled = True

        self._servo_enabled = True

    def servo_disable(self) -> None:
        if not self.servo_enabled:
            return
        self.servo.disable(sync=False)
        self._switch_controllers(
            activate=["scaled_joint_trajectory_controller"],
            deactivate=["forward_position_controller"],
        )

        # TODO(issue#X) ROS spin deadlockswith synchronous calls
        # HACK workaround for sync wait deadlock
        time.sleep(1.0)
        self.servo.__is_enabled = False

        self._servo_enabled = False

    def _switch_controllers(
        self, activate: Iterable[str], deactivate: Iterable[str], strict=True
    ) -> None:
        req = SwitchController.Request()
        req.activate_controllers = activate
        req.deactivate_controllers = deactivate
        req.strictness = (
            SwitchController.Request.STRICT
            if strict
            else SwitchController.Request.BEST_EFFORT
        )
        self.switch_controllers.call_async(req)

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
        self._wait_for_move_execution(cancel_after_secs)

    def pose_move(
        self,
        position: Optional[
            Union[Point, tuple[float, float, float], dict[str, float]]
        ] = None,
        quat_xyzw: Optional[
            Union[Quaternion, tuple[float, float, float, float], dict[str, float]]
        ] = None,
        pose: Optional[Union[PoseStamped, Pose]] = None,
        max_vel: float = 1.0,
        max_accel: float = 1.0,
        cartesian: bool = True,
        cartesian_max_step: float = 0.01,
        cartesian_fraction_threshold: float = 0.0,
        cartesian_jump_threshold: float = 0.0,
        cartesian_avoid_collisions: bool = True,
        cancel_after_secs: float = 0.0,
    ) -> None:
        self.moveit2.max_velocity = max_vel
        self.moveit2.max_acceleration = max_accel
        self.moveit2.cartesian_avoid_collisions = cartesian_avoid_collisions
        self.moveit2.cartesian_jump_threshold = cartesian_jump_threshold

        self.node.get_logger().info(
            f"Moving to {{position: {position}, quat: {quat_xyzw}}}, max_vel: {max_vel:.2f}, max_accel: {max_accel:.2f}}}"
        )
        self.moveit2.move_to_pose(
            pose=pose,
            position=position,
            quat_xyzw=quat_xyzw,
            cartesian=cartesian,
            cartesian_max_step=cartesian_max_step,
            cartesian_fraction_threshold=cartesian_fraction_threshold,
        )
        self._wait_for_move_execution(cancel_after_secs)

    def gripper_move(
        self, width: Optional[float] = None, action: Optional[str] = None
    ) -> None:
        if action is not None:
            self.node.get_logger().info(f"Using gripper action: {action}")
            match action.lower():
                case "open":
                    self.gripper_interface.open(skip_if_noop=True)
                case "close":
                    self.gripper_interface.close(skip_if_noop=True)
                case "toggle":
                    self.gripper_interface.toggle()
                case _:
                    raise ValueError(
                        "Invalid action. Use 'open', 'close', or 'toggle'."
                    )
            self.node.get_logger().info(f"Waiting for action: {action}")
            self._wait_for_gripper_execution()
            return

        if width is None:
            self.node.get_logger().info("No width provided, ignoring gripper action.")
            return

        min = robot.CLOSED_GRIPPER_JOINT_POSITIONS.left
        max = robot.OPEN_GRIPPER_JOINT_POSITIONS.left
        if not (min <= width <= max):
            raise ValueError(f"Width must be between {min} and {max}.")
        self.node.get_logger().info(f"Moving gripper to width: {width:.3f}.")
        self.gripper_interface.move_to_position(width)
        self._wait_for_gripper_execution()

    def servo_move(
        self,
        linear: tuple[float, float, float],
        angular: tuple[float, float, float] = (0.0, 0.0, 0.0),
    ) -> None:
        if not self.servo_enabled:
            self.node.get_logger().warn("Enable servo before moving. Ignoring.")
            return
        self.servo.servo(linear=linear, angular=angular, enable_if_disabled=False)

    def servo_jog(
        self,
        joint_names: tuple[str, ...],
        velocities: tuple[float, ...] = tuple(),
    ) -> None:
        if not self.servo_enabled:
            self.node.get_logger().warn("Enable servo before moving. Ignoring.")
            return
        self.servo.servo_jog(
            joint_names=joint_names, velocities=velocities, enable_if_disabled=False
        )

    def _wait_for_move_execution(self, cancel_after_secs: float = 0.0) -> None:
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

    def _wait_for_gripper_execution(self) -> None:
        # TODO(issue#39) Due to a unknown bug in pymoveit2, we got a deadlock with the current configuration
        # self.gripper_interface.wait_until_executed()
        # This is a temporal workaround to avoid the deadlock.
        time.sleep(0.1)
