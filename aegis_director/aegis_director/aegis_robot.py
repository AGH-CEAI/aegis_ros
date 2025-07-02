from typing import List

MOVE_GROUP_ARM: str = "aegis_arm"
MOVE_GROUP_GRIPPER: str = "aegis_manipulator"

prefix: str = ""

OPEN_GRIPPER_JOINT_POSITIONS: List[float] = [0.025, 0.025]
CLOSED_GRIPPER_JOINT_POSITIONS: List[float] = [0.0, 0.0]


def joint_names(prefix: str = prefix) -> List[str]:
    return [
        f"{prefix}shoulder_pan_joint",
        f"{prefix}shoulder_lift_joint",
        f"{prefix}elbow_joint",
        f"{prefix}wrist_1_joint",
        f"{prefix}wrist_2_joint",
        f"{prefix}wrist_3_joint",
    ]


def base_link_name(prefix: str = prefix) -> str:
    return f"{prefix}base_link"


def end_effector_name(prefix: str = prefix) -> str:
    return f"{prefix}robotiq_hande_end"


def gripper_joint_names(prefix: str = prefix) -> List[str]:
    return [
        f"{prefix}robotiq_hande_left_joint",
        f"{prefix}robotiq_hande_right_joint",
    ]
