import time
import hydra
import numpy as np

import gym


class RobotEnv(gym.Env):
    def __init__(self,
                 robot,
                 camera_manager_cfg,
                 workspace_limits):
        """
        :param robot:
        :param workspace_limits: workspace bounding box [[x_min, y_min, z_min], [x_max, y_max, z_max]]
        """
        self.robot = robot
        self.workspace_limits = workspace_limits

        self.camera_manager = hydra.utils.instantiate(camera_manager_cfg)

    def reset(self):
        """
        Reset robot to neutral position.
        """
        self.robot.move_to_neutral()
        return self._get_obs()

    def _get_obs(self):
        """
        :return: dictionary with image obs and state obs
        """
        obs = self.camera_manager.get_images()
        obs['robot_state'] = self.robot.get_state()
        return obs

    def step(self, action):
        """
        Execute one action on the robot.
        :param action: cartesian action tuple (position, orientation, gripper_action)
        :return: obs, reward, done, info
        """
        if action is None:
            return self._get_obs(), 0, False, {}
        assert isinstance(action, tuple) and len(action) == 3

        target_pos, target_orn, gripper_action = action
        target_pos = self._restrict_workspace(target_pos)
        self.robot.move_async_cart_pos_abs_ptp(target_pos, target_orn)

        if gripper_action == 1:
            self.robot.open_gripper()
        elif gripper_action == -1:
            self.robot.close_gripper()
        else:
            raise ValueError

        obs = self._get_obs()
        return obs, 0, False, {}

    def _restrict_workspace(self, target_pos):
        """
        :param target_pos: cartesian target position
        :return: clip target_pos at workspace limits
        """
        return np.clip(target_pos, self.workspace_limits[0], self.workspace_limits[1])

    def render(self, mode='human'):
        if mode == 'human':
            self.camera_manager.render()
