import time

import numpy as np
import hydra.utils

from robot_io.actions.rel_action_control import RelActionControl
from robot_io.robot_interface.base_robot_interface import BaseRobotInterface
from frankx import Affine, JointMotion, Robot, WaypointMotion, Waypoint, \
    ImpedanceMotion, JointWaypointMotion
from _frankx import NetworkException

from robot_io.utils.frankx_utils import to_affine
from robot_io.utils.utils import pos_orn_to_matrix, get_git_root, ReferenceType
import logging
log = logging.getLogger(__name__)

# FrankX needs continuous Euler angles around TCP, as the trajectory generation works in the Euler space.
# internally, FrankX expects orientations with the z-axis facing up, but to be consistent with other
# robot interfaces we transform the TCP orientation such that the z-axis faces down.
NE_T_EE = EE_T_NE = Affine(0, 0, 0, 0, 0, np.pi)

# align the output of force torque reading with the EE frame
WRENCH_FRAME_CONV = np.diag([-1, 1, 1, -1, 1, 1])  # np.eye(6)


class PandaFrankXInterface(BaseRobotInterface):
    """
    Robot control interface for Franka Emika Panda robot to be used on top of this Frankx fork
    (https://github.com/lukashermann/frankx)

    Args:
        fci_ip: IPv4 address of Franka Control Interface (FCI).
        urdf_path: URDF of panda robot (change default config e.g. when mounting different fingers).
        neutral_pose: Joint angles in rad.
        ll: Lower joint limits in rad.
        ul: Upper joint limits in rad.
        ik: Config of the inverse kinematic solver.
        workspace_limits: Workspace limits defined as a bounding box or as hollow cylinder.
        libfranka_params: DictConfig of params for libfranka.
        frankx_params: DictConfig of general params for Frankx.
        impedance_params: DictConfig of params for Frankx impedance motion.
        rel_action_params: DictConfig of params for relative action control.
        gripper_params: DictConfig of params for Frankx gripper.
    """
    def __init__(self,
                 fci_ip,
                 urdf_path,
                 neutral_pose,
                 ll,
                 ul,
                 ik,
                 workspace,
                 libfranka_params,
                 frankx_params,
                 impedance_params,
                 rel_action_params,
                 gripper):
        self.name = "panda"
        self._neutral_pose = neutral_pose

        # robot
        self._robot = Robot(fci_ip, urdf_path=(get_git_root(__file__) / urdf_path).as_posix())
        self._robot.recover_from_errors()
        self._robot.set_default_behavior()
        self._libfranka_params = libfranka_params
        self.set_robot_params(libfranka_params, frankx_params)

        # impedance
        self._impedance_params = impedance_params

        self._rel_action_converter = RelActionControl(ll=ll, ul=ul, workspace=workspace, **rel_action_params)

        self._motion_thread = None
        self._current_motion = None

        self._gripper = hydra.utils.instantiate(gripper)
        self.open_gripper(blocking=True)

        # F_T_NE is the transformation from nominal end-effector (NE) frame to flange (F) frame.
        F_T_NE = np.array(self._robot.read_once().F_T_NE).reshape((4, 4)).T
        self._ik_solver = hydra.utils.instantiate(ik, F_T_NE=F_T_NE)

        self._reference_type = ReferenceType.ABSOLUTE
        super().__init__(ll=ll, ul=ul)

    def __del__(self):
        self.abort_motion()

    def set_robot_params(self, libfranka_params, frankx_params):
        # params of libfranka
        self._robot.set_collision_behavior(libfranka_params.contact_torque_threshold,
                                           libfranka_params.collision_torque_threshold,
                                           libfranka_params.contact_force_threshold,
                                           libfranka_params.collision_force_threshold)
        self._robot.set_joint_impedance(libfranka_params.franka_joint_impedance)

        # params of frankx
        self._robot.velocity_rel = frankx_params.velocity_rel
        self._robot.acceleration_rel = frankx_params.acceleration_rel
        self._robot.jerk_rel = frankx_params.jerk_rel

    def move_to_neutral(self):
        return self.move_joint_pos(self._neutral_pose)

    def move_cart_pos(self, target_pos, target_orn, ref="abs", path="ptp", blocking=True, impedance=False):
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

        if blocking:
            if impedance:
                log.error("Impedance currently not implemented for synchronous motions.")
                raise NotImplementedError

            self.abort_motion()
            if path == "ptp":
                joint_positions = self._inverse_kinematics(target_pos, target_orn)
                success = self._robot.move(JointMotion(joint_positions))
            elif path == "lin":
                target_pose = to_affine(target_pos, target_orn) * NE_T_EE
                self._current_motion = WaypointMotion([Waypoint(target_pose)])
                success = self._robot.move(self._current_motion)
            else:
                raise ValueError

            if not success:
                self._robot.recover_from_errors()
            return success
        else:
            if path == "ptp":
                if impedance:
                    log.error("Impedance currently not implemented for asynchronous ptp motions.")
                    raise NotImplementedError
                joint_positions = self._inverse_kinematics(target_pos, target_orn)
                return self._frankx_async_joint_motion(joint_positions)
            elif path == "lin":
                if impedance:
                    return self._frankx_async_impedance_motion(target_pos, target_orn)
                else:
                    return self._frankx_async_lin_motion(target_pos, target_orn)
            else:
                raise ValueError

    def move_joint_pos(self, joint_positions, blocking=True):
        self._reference_type = ReferenceType.JOINT
        if blocking:
            self.abort_motion()
            success = self._robot.move(JointMotion(joint_positions))
            if not success:
                self._robot.recover_from_errors()
            return success
        else:
            return self._frankx_async_joint_motion(joint_positions)

    def abort_motion(self):
        if self._current_motion is not None:
            self._current_motion.stop()
            self._current_motion = None
        if self._motion_thread is not None:
        #     # self.motion_thread.stop()
            self._motion_thread.join()
            self._motion_thread = None
        while 1:
            try:
                self._robot.recover_from_errors()
                break
            except NetworkException:
                time.sleep(0.01)
                continue

    def get_state(self):
        if self._current_motion is None:
            _state = self._robot.read_once()
        else:
            _state = self._current_motion.get_robot_state()
        pos, orn = self.get_tcp_pos_orn()

        state = {"tcp_pos": pos,
                 "tcp_orn": orn,
                 "joint_positions": np.array(_state.q),
                 "gripper_opening_width": self._gripper.width(),
                 "force_torque": WRENCH_FRAME_CONV @ np.array(_state.K_F_ext_hat_K),
                 "contact": np.array(_state.cartesian_contact)}
        return state

    def get_tcp_pos_orn(self):
        if self._current_motion is None:
            pose = self._robot.current_pose() * EE_T_NE
        else:
            pose = self._current_motion.current_pose() * EE_T_NE
            while np.all(pose.translation() == 0):
                pose = self._current_motion.current_pose() * EE_T_NE
                time.sleep(0.01)
        pos, orn = np.array(pose.translation()), np.array(pose.quaternion())
        return pos, orn

    def get_tcp_pose(self):
        return pos_orn_to_matrix(*self.get_tcp_pos_orn())

    def open_gripper(self, blocking=False):
        self._gripper.open(blocking)

    def close_gripper(self, blocking=False):
        self._gripper.close(blocking)

    def _frankx_async_joint_motion(self, joint_positions):
        """
        Do not call this directly.

        Args:
            joint_positions:

        Returns:
        """
        if self._is_active(JointWaypointMotion):
            self._current_motion.set_next_target(joint_positions)
        else:
            if self._current_motion is not None:
                self.abort_motion()
            self._current_motion = JointWaypointMotion([joint_positions], return_when_finished=False)
            self._motion_thread = self._robot.move_async(self._current_motion)
        # TODO: how to handle async return?
        return True

    def _frankx_async_impedance_motion(self, target_pos, target_orn):
        """
        Start new async impedance motion. Do not call this directly.

        Args:
            target_pos: (x,y,z)
            target_orn: quaternion (x,y,z,w) | euler_angles (α,β,γ)
        """
        target_pose = to_affine(target_pos, target_orn) * NE_T_EE
        if self._is_active(ImpedanceMotion):
            self._current_motion.set_target(target_pose)
        else:
            if self._current_motion is not None:
                self.abort_motion()
            self._current_motion = self._new_impedance_motion()
            self._current_motion.set_target(target_pose)
            self._motion_thread = self._robot.move_async(self._current_motion)
        # TODO: how to handle async return?
        return True

    def _new_impedance_motion(self):
        """
        Create new frankx impedance motion with the params specified in config file.

        Returns:
            Impedance motion object.
        """
        if self._impedance_params.use_nullspace:
            return ImpedanceMotion(self._impedance_params.translational_stiffness,
                                   self._impedance_params.rotational_stiffness,
                                   self._impedance_params.nullspace_stiffness,
                                   self._impedance_params.q_d_nullspace,
                                   self._impedance_params.damping_xi)
        else:
            return ImpedanceMotion(self._impedance_params.translational_stiffness,
                                   self._impedance_params.rotational_stiffness)

    def _frankx_async_lin_motion(self, target_pos, target_orn):
        """
        Start new Waypaint motion without impedance. Do not call this directly.

        Args:
            target_pos: (x,y,z)
            target_orn: quaternion (x,y,z,w) | euler_angles (α,β,γ)
        """
        target_pose = to_affine(target_pos, target_orn) * NE_T_EE
        if self._is_active(WaypointMotion):
            self._current_motion.set_next_waypoint(Waypoint(target_pose))
        else:
            if self._current_motion is not None:
                self.abort_motion()
            self._current_motion = WaypointMotion([Waypoint(target_pose), ], return_when_finished=False)
            self._motion_thread = self._robot.move_async(self._current_motion)
        # TODO: how to handle async return?
        return True

    def _inverse_kinematics(self, target_pos, target_orn):
        """
        Find inverse kinematics solution with the ik solver specified in config file.

        Args:
            target_pos: cartesian target position (x,y,z).
            target_orn: cartesian target orientation, quaternion (x,y,z,w) | euler_angles (α,β,γ).

        Returns:
            Target joint angles in rad.
        """
        current_q = self.get_state()['joint_positions']
        new_q = self._ik_solver.inverse_kinematics(target_pos, target_orn, current_q)
        return new_q

    def _is_active(self, motion):
        """Returns True if there is a currently active motion with the same type as motion."""
        return self._current_motion is not None and isinstance(self._current_motion, motion) and self._motion_thread.is_alive()

    def visualize_external_forces(self, canvas_width=500):
        """
        Display the external forces (x,y,z) and torques (a,b,c) of the tcp frame.

        Args:
            canvas_width: Display width in pixel.
        """
        contact = np.array(self._libfranka_params.contact_force_threshold)
        collision = np.array(self._libfranka_params.collision_force_threshold)
        self._visualize_external_forces(contact, collision, canvas_width)


@hydra.main(config_path="../../conf", config_name="panda_teleop.yaml")
def main(cfg):
    robot = hydra.utils.instantiate(cfg.robot)
    robot.move_to_neutral()
    robot.close_gripper()
    time.sleep(1)
    print(robot.get_tcp_pose())
    exit()
    # print(robot.get_state()["gripper_opening_width"])
    # time.sleep(2)
    # robot.open_gripper()
    # time.sleep(1)
    # exit()
    # pos, orn = robot.get_tcp_pos_orn()
    # pos[0] += 0.2
    # pos[2] -= 0.1
    # # pos[2] -= 0.05
    # print("move")
    # robot.move_cart_pos_abs_ptp(pos, orn)
    # time.sleep(5)
    # print("done!")


if __name__ == "__main__":
    main()
