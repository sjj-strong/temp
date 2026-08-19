"""Joint-name constants shared across the package.

Vendored from ur_teleop/ur_teleop/config.py@6f202d2 — kept local so this
package never imports ur_teleop (requirement). Keep in sync with the original.
"""

UR_JOINT_NAMES = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
]
ALICIA_JOINT_NAMES = ["Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6"]
GRIPPER_JOINT = "Gripper"
UR_GRIPPER_JOINT = "robotiq_85_left_knuckle_joint"
