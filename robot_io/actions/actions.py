from dataclasses import dataclass
from typing import Union

import numpy as np


@dataclass
class Action:
    """
    Class for sending cartesian actions to the robot.
    Due to differences in the implementation of the robot controllers, not all action types are available for every
    robot.

    Args:
        target_pos: The target position, either absolute or relative.
        target_orn: The target orientation, either absolute or relative, as quaternion or euler angles.
        gripper_action: Binary gripper action. 1 -> open, -1 -> close.
        ref: "abs" (absolute w.r.t base frame) | "rel" (relative w.r.t. tcp frame) | "rel_world" (relative w.r.t. base frame)
        path: "ptp" (motion linear in joint space) | "lin" (motion linear in cartesian space)
        blocking: If True, block until the action is executed.
        impedance: If True, use impedance control (compliant robot). Typically less precise, but safer.
    """
    target_pos: Union[tuple, np.ndarray]
    target_orn: Union[tuple, np.ndarray]
    gripper_action: int
    ref: str = "abs"  # "abs" | "rel" | "rel_world"
    path: str = "ptp"  # "ptp" | "lin"

    blocking: bool = False
    impedance: bool = False


if __name__ == "__main__":
    a = Action((0, 0, 0), (0, 0, 0, 1), 1)
    a.blocking = True
    print(a)
