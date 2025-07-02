import time
import numpy as np

import rclpy
from aegis_director.robot_director import RobotDirector
from aegis_director.utils import quaternion_to_euler, euler_to_quaternion


def main(args=None):
    rclpy.init(args=args)
    director = RobotDirector(synchronous=True)
    node = director.node
    cancel_after_s = 5.0

    node.get_logger().info("Joint states: {}".format(director.get_joint_states()))

    joint_positions = {
        "shoulder_pan_joint": np.deg2rad(-8.0),
        "shoulder_lift_joint": np.deg2rad(-90.0),
        "elbow_joint": np.deg2rad(90.0),
        "wrist_1_joint": np.deg2rad(-100.0),
        "wrist_2_joint": np.deg2rad(-90.0),
        "wrist_3_joint": np.deg2rad(0.0),
    }

    director.gripper_move(action="open")
    node.get_logger().info("Joint states: {}".format(director.get_joint_states()))

    node.get_logger().info("Sleeping for 3 seconds...")
    time.sleep(3)

    director.joint_move(
        joint_positions=joint_positions,
        max_vel=1.0,
        max_accel=1.0,
        cancel_after_secs=cancel_after_s,
    )

    director.gripper_move(action="close")
    node.get_logger().info("Joint states: {}".format(director.get_joint_states()))

    node.get_logger().info("TCP pose: {}".format(director.get_tcp_pose()))

    director.gripper_move(width=0.01)
    node.get_logger().info("Joint states: {}".format(director.get_joint_states()))

    start_pose = director.get_tcp_pose()
    pos = start_pose["position"]
    quat = start_pose["orientation"]

    new_pos = pos + np.array([-0.1, -0.1, 0.1])
    ori_rpy = quaternion_to_euler(quat)
    ori_rpy[2] += np.deg2rad(90.0)  # Rotate around Z-axis by 90 degrees
    ori_quat = euler_to_quaternion(*ori_rpy)

    node.get_logger().info("Sleeping for 3 seconds...")
    time.sleep(3)

    director.pose_move(
        position=new_pos,
        quat_xyzw=ori_quat,
        cancel_after_secs=cancel_after_s,
    )

    director.gripper_move(action="toggle")
    node.get_logger().info("Joint states: {}".format(director.get_joint_states()))

    node.get_logger().info("Sleeping for 3 seconds...")
    time.sleep(3)

    joint_positions = {
        "shoulder_pan_joint": np.deg2rad(0.0),
        "shoulder_lift_joint": np.deg2rad(0.0),
        "elbow_joint": np.deg2rad(0.0),
        "wrist_1_joint": np.deg2rad(0.0),
        "wrist_2_joint": np.deg2rad(0.0),
        "wrist_3_joint": np.deg2rad(0.0),
    }
    director.joint_move(
        joint_positions=joint_positions,
        max_vel=1.0,
        max_accel=1.0,
        cancel_after_secs=cancel_after_s,
    )

    director.gripper_move(width=0.025)
    node.get_logger().info("Joint states: {}".format(director.get_joint_states()))

    rclpy.shutdown()
    exit(0)


if __name__ == "__main__":
    main()
