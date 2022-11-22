import socket
import time

import hydra
import numpy as np
from math import pi
from robot_io.robot_interface.base_robot_interface import BaseRobotInterface, GripperState
from robot_io.utils.utils import pos_orn_to_matrix, euler_to_quat, quat_to_euler, \
    matrix_to_pos_orn, xyz_to_zyx

import logging

from robot_io.workspace.workspace import BoxWorkspace

log = logging.getLogger(__name__)

JAVA_JOINT_MODE = 0
JAVA_CARTESIAN_MODE_REL_PTP = 1
JAVA_CARTESIAN_MODE_ABS_PTP = 2
JAVA_CARTESIAN_MODE_REL_LIN = 3
JAVA_CARTESIAN_MODE_ABS_LIN = 4
JAVA_SET_PROPERTIES = 5
JAVA_GET_INFO = 6
JAVA_INIT = 7
JAVA_ABORT_MOTION = 8
JAVA_SET_FRAME = 9


# iiwa TCP frames
TCP_SHORT_FINGER = 20
TCP = 21


class IIWAInterface(BaseRobotInterface):
    def __init__(self,
                 gripper,
                 workspace,
                 host="localhost",
                 port=50100,
                 joint_vel=0.1,
                 gripper_rot_vel=0.3,
                 joint_acc=0.3,
                 cartesian_vel=100,
                 cartesian_acc=300,
                 tcp_name=TCP_SHORT_FINGER,
                 neutral_pose=(0.5, 0, 0.25, pi, 0, pi / 2)):
        """
        :param host: "localhost"
        :param port: default port is 50100
        :param joint_vel: max velocities of joint 1-6, range [0, 1], for PTP/joint motions
        :param gripper_rot_vel: max velocities of joint 7, , range [0, 1], for PTP/joint motions
        :param joint_acc: max acceleration of joint 1-7, range [0,1], for PTP/joint motions
        :param cartesian_vel: max translational and rotational velocity of EE, in mm/s, for LIN motions
        :param cartesian_acc: max translational and rotational acceleration of EE, in mm/s**2, for LIN motions
        :param tcp_name: name of tcp frame in Java RoboticsAPI.data.xml
        """
        self.name = "iiwa"
        # TODO: use workspace class
        assert isinstance(workspace, BoxWorkspace)
        self._workspace_limits = workspace.limits
        self._version_counter = 1
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.bind((host, port + 500))
        self._socket.connect((host, port))
        self._use_impedance = True
        self._send_init_message()
        self._joint_vel = joint_vel
        self._gripper_rot_vel = gripper_rot_vel
        self._joint_acc = joint_acc
        self._cartesian_vel = cartesian_vel
        self._cartesian_acc = cartesian_acc
        self._tcp_name = tcp_name
        self.set_properties(joint_vel, gripper_rot_vel, joint_acc, cartesian_vel, cartesian_acc, self._use_impedance,
                            self._workspace_limits, tcp_name)
        self._neutral_pose = np.array(neutral_pose)
        self._gripper = hydra.utils.instantiate(gripper)
        self._gripper_state = GripperState.OPEN
        self._gripper.open_gripper()
        super().__init__()

    def move_to_neutral(self):
        if len(self._neutral_pose) == 6:
            target_pos = self._neutral_pose[:3]
            target_orn = euler_to_quat(self._neutral_pose[3:6])
            self.move_cart_pos(target_pos, target_orn, ref="abs", path="ptp")
        elif len(self._neutral_pose) == 7:
            self.move_joint_pos(self._neutral_pose)

    def get_state(self):
        msg = np.array([self._version_counter], dtype=np.int32).tobytes()
        msg += np.array([JAVA_GET_INFO], dtype=np.int16).tobytes()
        state = self._send_recv_message(msg, 184)
        return self._create_info_dict(state)

    def get_tcp_pose(self):
        pos, orn = self.get_tcp_pos_orn()
        return pos_orn_to_matrix(pos, orn)

    def get_tcp_pos_orn(self):
        pos, orn = self.get_state()['tcp_pose'][0:3], self.get_state()['tcp_pose'][3:6]
        orn = euler_to_quat(orn)
        return pos, orn

    def move_cart_pos(self, target_pos, target_orn, ref="abs", path="ptp", blocking=True, impedance=False):
        if impedance != self._use_impedance:
            self._use_impedance = impedance
            self.set_properties(self._joint_vel, self._gripper_rot_vel, self._joint_acc, self._cartesian_vel,
                                self._cartesian_acc, impedance, self._workspace_limits, self._tcp_name)
        if ref == "abs" and path == "ptp":
            mode = JAVA_CARTESIAN_MODE_ABS_PTP
        elif ref == "abs" and path == "lin":
            mode = JAVA_CARTESIAN_MODE_ABS_LIN
        elif ref == "rel" and path == "ptp":
            mode = JAVA_CARTESIAN_MODE_REL_PTP
        elif ref == "rel" and path == "lin":
            mode = JAVA_CARTESIAN_MODE_REL_LIN
        else:
            raise ValueError
        pose = self._process_pose(target_pos, target_orn)
        msg = self._create_robot_msg(pose, mode)
        state = self._send_recv_message(msg, 184)
        if blocking:
            while not self.reached_position(target_pos, target_orn):
                time.sleep(0.1)

    def move_joint_pos(self, joint_positions, blocking=True):
        assert len(joint_positions) == 7
        joint_positions = np.array(joint_positions, dtype=np.float64)
        msg = self._create_robot_msg(joint_positions, JAVA_JOINT_MODE)
        state = self._send_recv_message(msg, 184)
        while not self.reached_joint_state(joint_positions):
            time.sleep(0.1)

    def abort_motion(self):
        msg = np.array([self._version_counter], dtype=np.int32).tobytes()
        msg += np.array([JAVA_ABORT_MOTION], dtype=np.int16).tobytes()
        return self._send_recv_message(msg, 188)

    def open_gripper(self, blocking=False):
        if self._gripper_state == GripperState.CLOSED:
            self._gripper.open_gripper()
            if blocking:
                # TODO: implement this properly
                time.sleep(1)
            self._gripper_state = GripperState.OPEN

    def close_gripper(self, blocking=False):
        if self._gripper_state == GripperState.OPEN:
            self._gripper.close_gripper()
            if blocking:
                # TODO: implement this properly
                time.sleep(1)
            self._gripper_state = GripperState.CLOSED

    @staticmethod
    def _create_info_dict(state):
        state[:3] *= 0.001
        return {'tcp_pose': state[:6], 'joint_positions': state[6:13], 'desired_tcp_pose': state[13:17],
                'force_torque': state[17:23]}

    def _send_recv_message(self, message, recv_msg_size):
        self._socket.send(bytes(message))
        reply, address = self._socket.recvfrom(4 * recv_msg_size)
        return np.frombuffer(reply, dtype=np.float64).copy()

    def _send_init_message(self):
        msg = np.array([self._version_counter], dtype=np.int32).tobytes()
        msg += np.array([JAVA_INIT], dtype=np.int16).tobytes()
        return self._send_recv_message(msg, 188)

    def set_properties(self, joint_vel, gripper_rot_vel, joint_acc, cartesian_vel, cartesian_acc, use_impedance,
                       workspace_limits, tcp_name):
        msg = np.array([self._version_counter], dtype=np.int32).tobytes()
        msg += np.array([JAVA_SET_PROPERTIES], dtype=np.int16).tobytes()
        msg += np.array([joint_vel], dtype=np.float64).tobytes()
        msg += np.array([gripper_rot_vel], dtype=np.float64).tobytes()
        msg += np.array([joint_acc], dtype=np.float64).tobytes()
        msg += np.array([cartesian_vel], dtype=np.float64).tobytes()
        msg += np.array([cartesian_acc], dtype=np.float64).tobytes()
        msg += np.array([use_impedance], dtype=np.int16).tobytes()
        msg += np.array([workspace_limits[0][0] * 1000], dtype=np.float64).tobytes()
        msg += np.array([workspace_limits[1][0] * 1000], dtype=np.float64).tobytes()
        msg += np.array([workspace_limits[0][1] * 1000], dtype=np.float64).tobytes()
        msg += np.array([workspace_limits[1][1] * 1000], dtype=np.float64).tobytes()
        msg += np.array([workspace_limits[0][2] * 1000], dtype=np.float64).tobytes()
        msg += np.array([workspace_limits[1][2] * 1000], dtype=np.float64).tobytes()
        msg += np.array([tcp_name], dtype=np.int16).tobytes()
        state = self._send_recv_message(msg, 188)

    def set_goal_frame(self, T_robot_goal, goal_workspace_limits):
        msg = np.array([self._version_counter], dtype=np.int32).tobytes()
        msg += np.array([JAVA_SET_FRAME], dtype=np.int16).tobytes()
        pos, orn = matrix_to_pos_orn(T_robot_goal)
        orn = xyz_to_zyx(quat_to_euler(orn))
        msg += np.array([pos[0] * 1000], dtype=np.float64).tobytes()
        msg += np.array([pos[1] * 1000], dtype=np.float64).tobytes()
        msg += np.array([pos[2] * 1000], dtype=np.float64).tobytes()
        msg += np.array([orn[0]], dtype=np.float64).tobytes()
        msg += np.array([orn[1]], dtype=np.float64).tobytes()
        msg += np.array([orn[2]], dtype=np.float64).tobytes()
        msg += np.array([goal_workspace_limits[0][0] * 1000], dtype=np.float64).tobytes()
        msg += np.array([goal_workspace_limits[1][0] * 1000], dtype=np.float64).tobytes()
        msg += np.array([goal_workspace_limits[0][1] * 1000], dtype=np.float64).tobytes()
        msg += np.array([goal_workspace_limits[1][1] * 1000], dtype=np.float64).tobytes()
        msg += np.array([goal_workspace_limits[0][2] * 1000], dtype=np.float64).tobytes()
        msg += np.array([goal_workspace_limits[1][2] * 1000], dtype=np.float64).tobytes()
        state = self._send_recv_message(msg, 188)

    @staticmethod
    def _process_pose(pos, orn):
        pos = np.array(pos, dtype=np.float64) * 1000
        orn = np.array(orn, dtype=np.float64)
        if len(orn) == 4:
            orn = quat_to_euler(orn)
        return np.concatenate([pos, orn])

    def _create_robot_msg(self, pose, mode):
        assert type(mode) == int
        msg = np.array([self._version_counter], dtype=np.int32).tobytes()
        msg += np.array([mode], dtype=np.int16).tobytes()
        for c in pose:
            msg += c.tobytes()
        msg += np.array([12345], dtype=np.int64).tobytes()
        return msg


if __name__ == "__main__":
    robot = IIWAInterface()
    robot.move_to_neutral()
    pos, orn = robot.get_tcp_pos_orn()
    pos[2] += 0.05
    robot.move_cart_pos(pos, orn, ref="abs", path="ptp")

