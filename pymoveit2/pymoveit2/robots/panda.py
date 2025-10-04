from typing import List

# Move group name for the Panda arm
MOVE_GROUP_ARM: str = "arm"

# Move group name for the Panda gripper
MOVE_GROUP_GRIPPER: str = "gripper"

def joint_names() -> List[str]:
    return ["panda_joint1",
            "panda_joint2",
            "panda_joint3",
            "panda_joint4",
            "panda_joint5",
            "panda_joint6",
            "panda_joint7",]

def gripper_joint_names() -> List[str]:
    return ["panda_finger_joint1",
            "panda_finger_joint2",]

def base_link_name() -> str:
    return "panda_link0"


def end_effector_name() -> str:
    return "panda_hand"

# def tip_link_name() -> str:
#     return "panda_link7"

