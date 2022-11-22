import time

import hydra
import numpy as np
import rtde_control
import rtde_receive
from scipy.spatial.transform.rotation import Rotation as R
from robot_io.actions.rel_action_control import RelActionControl
from robot_io.robot_interface.base_robot_interface import BaseRobotInterface
from robot_io.utils.utils import ReferenceType, rotvec_to_quat, quat_to_rotvec, euler_to_rotvec, pos_orn_to_matrix, \
    euler_to_quat, orn_to_matrix

import logging
log = logging.getLogger(__name__)


def to_ur_pose(pos, orn):
    if len(orn) == 4:
        rot_vec = quat_to_rotvec(orn)
    elif len(orn) == 3:
        rot_vec = euler_to_rotvec(orn)
    else:
        raise ValueError
    return np.concatenate([pos, rot_vec])


def wrench_world_to_tcp(force_torque, tcp_orn):
    """
    Convert the force-torque wrench from base frame to tcp frame.

    Args:
        force_torque: 6d force-torque wrench in base frame.
        tcp_orn: tcp orientation.

    Returns:
        6d force-torque wrench in tcp frame.
    """
    force_b = np.array(force_torque[:3])
    torque_b = np.array(force_torque[3:])
    tcp_R_base = np.linalg.inv(orn_to_matrix(tcp_orn))
    force_tcp = tcp_R_base @ force_b
    torque_tcp = tcp_R_base @ torque_b
    return np.concatenate([force_tcp, torque_tcp])


class URInterface(BaseRobotInterface):
    def __init__(self,
                 robot_ip,
                 gripper,
                 ll,
                 ul,
                 tcp_offset,
                 neutral_pose,
                 workspace,
                 cartesian_speed,
                 cartesian_acc,
                 joint_speed,
                 joint_acc,
                 servo_max_distance_threshold_pos,
                 servo_max_distance_threshold_orn,
                 contact_force_threshold,
                 control_time,
                 lookahead_time,
                 gain,
                 rel_action_params):
        self.name = "ur3"
        self._neutral_pose = neutral_pose
        self._reference_type = ReferenceType.ABSOLUTE
        self._is_servoing = False
        self._cartesian_speed = cartesian_speed
        self._cartesian_acc = cartesian_acc
        self._joint_speed = joint_speed
        self._joint_acc = joint_acc
        self._servo_max_distance_threshold_pos = servo_max_distance_threshold_pos
        self._servo_max_distance_threshold_orn = servo_max_distance_threshold_orn
        self._contact_force_threshold = np.array(contact_force_threshold)
        self._control_time = control_time  # [s] time when the command is controlling the robot
        self._lookahead_time = lookahead_time  # [s] smoothens the trajectory with this [0.03, 02]
        self._gain = gain  # proportional gain for following target position, range [100,2000]

        try:
            self._rtde_c = rtde_control.RTDEControlInterface(robot_ip)
            self._rtde_r = rtde_receive.RTDEReceiveInterface(robot_ip)
        except RuntimeError as e:
            if e.args == ('Timeout connecting to UR dashboard server.',):
                log.warning(f"\n\nProTip: Is the robot turned on and the network connection to {robot_ip} active?\n")
            elif e.args == ('ur_rtde: Failed to start control script, before timeout of 5 seconds',):
                log.warning("\n\nProTip: Is the robot activated & started in addition to being on?\n")
            else:
                log.warning("UR loading RTDE failed with message: %s", e.args)
            self._rtde_c = None
            self._rtde_r = None
            raise

        self._rtde_c.setTcp(tcp_offset)
        self._gripper = hydra.utils.instantiate(gripper)
        self.open_gripper(blocking=True)

        self._rel_action_converter = RelActionControl(ll=ll, ul=ul, workspace=workspace,
                                                      **rel_action_params)
        super().__init__(ll=ll, ul=ul)

    def __del__(self):
        if self._rtde_c is None:
            return
        self.abort_motion()
        self._rtde_c.stopScript()

    def move_to_neutral(self):
        return self.move_joint_pos(self._neutral_pose)

    def move_cart_pos(self, target_pos, target_orn, ref="abs", path="ptp", blocking=True, impedance=False):
        assert ref in ("abs", "rel", "rel_world")
        assert path in ("ptp", "lin")
        if impedance:
            log.error("Impedance currently not implemented.")
            raise NotImplementedError
        if self._reference_type == ReferenceType.JOINT:
            self.abort_motion()
        if ref == "abs":
            self._reference_type = ReferenceType.ABSOLUTE
        elif ref == "rel" or ref == "rel_world":
            control_frame = "tcp" if ref == "rel" else "world"
            target_pos, target_orn = self._rel_action_converter.to_absolute(target_pos,
                                                                            target_orn,
                                                                            self.get_state(),
                                                                            self._reference_type,
                                                                            control_frame=control_frame)
            self._reference_type = ReferenceType.RELATIVE
        else:
            raise ValueError
        pose = to_ur_pose(target_pos, target_orn)

        if blocking:
            self.abort_motion()
            if path == "ptp":
                self._rtde_c.moveJ_IK(pose, self._joint_speed, self._joint_acc, asynchronous=False)
            elif path == "lin":
                self._rtde_c.moveL(pose, self._cartesian_speed, self._cartesian_acc, asynchronous=False)
            else:
                raise ValueError
            self._is_servoing = False
        else:
            if path == "ptp":
                if self._is_servoing:
                    self.abort_motion()
                self._rtde_c.moveJ_IK(pose, self._joint_speed, self._joint_acc, asynchronous=True)
                self._is_servoing = False
            elif path == "lin":
                # for servoing, check that new pose is not too far away (otherwise robot would move too fast)
                if self._servoing_feasible(target_pos, target_orn):
                    if not self._is_servoing:
                        self.abort_motion()
                    # self.cartesian_speed, self.cartesian_acc is not used here
                    self._rtde_c.servoL(pose, self._cartesian_speed, self._cartesian_acc, self._control_time, self._lookahead_time, self._gain)
                    self._is_servoing = True
                else:
                    if self._is_servoing:
                        self.abort_motion()
                    self._rtde_c.moveL(pose, self._cartesian_speed, self._cartesian_acc, asynchronous=True)
                    self._is_servoing = False
            else:
                raise ValueError
        # TODO: return success
        return True

    def abort_motion(self):
        if self._reference_type == ReferenceType.JOINT:
            self._rtde_c.stopJ(0.5)
        elif self._is_servoing:
            self._rtde_c.servoStop(0.5)
        else:
            self._rtde_c.stopL(0.5)

    def move_joint_pos(self, joint_positions, blocking=True):
        self._reference_type = ReferenceType.JOINT
        self.abort_motion()
        return self._rtde_c.moveJ(joint_positions, self._joint_speed, self._joint_acc, asynchronous=not blocking)

    def get_state(self):
        pos, orn = self.get_tcp_pos_orn()
        state = {"tcp_pos": pos,
                 "tcp_orn": orn,
                 "joint_positions": np.array(self._rtde_r.getActualQ()),
                 "gripper_opening_width": self._gripper.get_opening_width(),
                 "force_torque": wrench_world_to_tcp(self._rtde_r.getActualTCPForce(), orn)}
        contact = np.zeros(6)
        contact[np.where(np.abs(state["force_torque"]) > self._contact_force_threshold)] = 1
        state["contact"] = contact
        return state

    def get_tcp_pos_orn(self):
        pose = np.array(self._rtde_r.getActualTCPPose())
        pos, orn = pose[:3], rotvec_to_quat(pose[3:])
        return pos, orn

    def get_tcp_pose(self):
        return pos_orn_to_matrix(*self.get_tcp_pos_orn())

    def open_gripper(self, blocking=False):
        self._gripper.open_gripper(blocking)

    def close_gripper(self, blocking=False):
        self._gripper.close_gripper(blocking)

    def visualize_external_forces(self, canvas_width=500):
        """
        Display the external forces (x,y,z) and torques (a,b,c) of the tcp frame.

        Args:
            canvas_width: Display width in pixel.
        """
        contact = self._contact_force_threshold
        collision = np.array([50, 50, 50, 50, 50, 50])
        self._visualize_external_forces(contact, collision, canvas_width)

    def _servoing_feasible(self, target_pos, target_orn):
        """
        Check if current pose is close enough to target pose to perform servoing motion.

        Args:
            target_pos: (x,y,z)
            target_orn: Quaternion (x,y,z,w) | Euler_angles (α,β,γ).

        Returns:
            True if close enough to target.
        """
        if len(target_orn) == 3:
            target_orn = euler_to_quat(target_orn)
        curr_pos, curr_orn = self.get_tcp_pos_orn()
        pos_distance = np.linalg.norm(target_pos - curr_pos)
        orn_distance = np.linalg.norm((R.from_quat(target_orn) * R.from_quat(curr_orn).inv()).as_rotvec())
        return pos_distance < self._servo_max_distance_threshold_pos and orn_distance < self._servo_max_distance_threshold_orn


if __name__ == "__main__":
    hydra.initialize("../../conf/robot/")
    cfg = hydra.compose("ur_interface.yaml")
    robot = hydra.utils.instantiate(cfg)

    robot.move_to_neutral()

    home_pos, home_orn = robot.get_tcp_pos_orn()
    print("home:", home_pos, home_orn)

    orn = np.array([0, 0, 0])
    for i in range(100):
        pos = np.random.uniform(-0.02, 0.02, 3)
        blocking = np.random.choice([True, False])
        path = np.random.choice(["ptp", "lin"])
        robot.move_cart_pos(pos, orn, ref="rel", blocking=blocking, path=path)
        print(pos, blocking, path)
        print(robot.get_tcp_pos_orn())
        time.sleep(np.random.random())

    # pos = np.array([-0.01, 0, 0])
    # robot.move_cart_pos(pos, orn, ref="rel", blocking=False, path="ptp")

    print("done!")
    print(robot.get_state())
