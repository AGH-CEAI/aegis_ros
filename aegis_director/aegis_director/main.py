import time
import numpy as np

import rclpy
from rclpy.node import Node
from aegis_director.robot_director import RobotDirector
from aegis_director.utils import quaternion_to_euler, euler_to_quaternion


def main(args=None):
    rclpy.init(args=args)
    node = Node("director")
    director = RobotDirector(node=node, synchronous=True)

    node.get_logger().info("Joint states: {}".format(director.get_joint_states()))

    joint_positions = {
        "shoulder_pan_joint": np.deg2rad(-8.0),
        "shoulder_lift_joint": np.deg2rad(-90.0),
        "elbow_joint": np.deg2rad(90.0),
        "wrist_1_joint": np.deg2rad(-100.0),
        "wrist_2_joint": np.deg2rad(-90.0),
        "wrist_3_joint": np.deg2rad(0.0),
    }

    cancel_after_s = 0.0
    director.joint_move(
        joint_positions=joint_positions,
        max_vel=1.0,
        max_accel=1.0,
        cancel_after_secs=cancel_after_s,
    )

    node.get_logger().info("TCP pose: {}".format(director.get_tcp_pose()))

    node.get_logger().info("Sleeping for 3 seconds...")
    time.sleep(3)

    start_pose = director.get_tcp_pose()
    pos = start_pose["position"]
    quat = start_pose["orientation"]

    new_pos = pos + np.array([-0.1, -0.1, 0.1])
    ori_rpy = quaternion_to_euler(quat)
    ori_rpy[2] += np.deg2rad(90.0)  # Rotate around Z-axis by 90 degrees
    ori_quat = euler_to_quaternion(*ori_rpy)

    director.pose_move(
        position=new_pos,
        quat_xyzw=ori_quat,
    )

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

    rclpy.shutdown()
    node.destroy_node()
    exit(0)


if __name__ == "__main__":
    main()
