import time
import numpy as np

import rclpy
from rclpy.node import Node
from aegis_director.robot_director import RobotDirector 

def main(args=None):
    rclpy.init(args=args)
    node = Node("director")
    director = RobotDirector(node=node, synchronous=True)
    
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
    
    # Sleep python for 3 seconds
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
