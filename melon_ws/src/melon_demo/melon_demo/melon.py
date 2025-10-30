from typing import List

MOVE_GROUP_ARM: str = "panda_arm"
MOVE_GROUP_GRIPPER: str = "hand"

OPEN_GRIPPER_JOINT_POSITIONS: List[float] = [0.04, 0.04]
CLOSED_GRIPPER_JOINT_POSITIONS: List[float] = [0.0, 0.0]

def joint_names() -> List[str]:
    return [
        "fr3_joint1",
        "fr3_joint2",
        "fr3_joint3",
        "fr3_joint4",
        "fr3_joint5",
        "fr3_joint6",
        "fr3_joint7",
    ]

def base_link_name() -> str:
    return "base_link"

def end_effector_name() -> str:
    return "fr3_hand_tcp"

def gripper_joint_names() -> List[str]:
    return [
        "fr3_finger_joint1",
        "fr3_finger_joint2",
    ]