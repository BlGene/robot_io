import time
from pathlib import Path

import hydra
import numpy as np
from robot_io.utils.utils import ReferenceType
from robot_io.utils.utils import restrict_workspace

from robot_io.utils.utils import FpsController, to_relative_action_dict, to_relative_action_pos_dict

N_DIGITS = 6


def get_ep_start_end_ids(path):
    return np.sort(np.load(Path(path) / "ep_start_end_ids.npy"), axis=0)


def get_frame(path, i):
    filename = Path(path) / f"frame_{i:0{N_DIGITS}d}.npz"
    return np.load(filename, allow_pickle=True)


def reset(env, path, i):
    data = get_frame(path, i)
    robot_state = data['robot_state'].item()
    gripper_state = "open" if robot_state["gripper_opening_width"] > 0.07 else "closed"
    env.reset(target_pos=robot_state["tcp_pos"], target_orn=robot_state["tcp_orn"], gripper_state=gripper_state)


def get_action(path, i, use_rel_actions=False):
    frame = get_frame(path, i)
    action = frame["action"].item()
    if use_rel_actions:
        prev_action = get_frame(path, i - 1)['action'].item()
        return to_relative_action_dict(prev_action, action)
    else:
        return action

def get_action_pos(path, i, use_rel_actions=False):
    frame = get_frame(path, i)
    pos = frame["robot_state"].item()
    action = frame["action"].item()

    if use_rel_actions:
        next_pos = get_frame(path, i + 1)['robot_state'].item()
        gripper_action = action["motion"][-1]
        return to_relative_action_pos_dict(pos, next_pos, gripper_action)
    else:
        return frame


@hydra.main(config_path="../conf", config_name="replay_recorded_trajectory")
def main(cfg):
    """
    Replay a recorded trajectory, either with absolute actions or relative actions.

    Args:
        cfg: Hydra config
    """
    robot = hydra.utils.instantiate(cfg.robot)
    # env = hydra.utils.instantiate(cfg.env, robot=robot)

    use_rel_actions = cfg.use_rel_actions
    ep_start_end_ids = get_ep_start_end_ids(cfg.load_dir)
    # fps = FpsController(cfg.freq)

    # env.reset()
    reference_type=ReferenceType.JOINT
    # prev_action, curent_obs
    frame = get_frame(cfg.load_dir, 0)
    next_frame = get_frame(cfg.load_dir, 1)

    robot_state = frame["robot_state"].item()
    pos = robot_state["tcp_pos"]
    for start_idx, end_idx in ep_start_end_ids:
        # reset(env, cfg.load_dir, start_idx)
        for i in range(start_idx + 1, end_idx + 1):
            # Absolute actions
            # prev_action, curent_obs
            frame = get_frame(cfg.load_dir, i)
            next_frame = get_frame(cfg.load_dir, i + 1)

            robot_state = frame["robot_state"].item()
            action = next_frame["action"].item()
            target_pos, target_orn, gripper_action = action['motion']
            target_pos = restrict_workspace(cfg.robot.workspace_limits, target_pos)

            # Relative actions converted to absolute
            relative_action = get_action_pos(cfg.load_dir, i + 1, True)
            rel_target_pos = relative_action["motion"][0]
            rel_target_orn = relative_action["motion"][1]
            converted_pos, converted_orn = robot.rel_action_converter.to_absolute(rel_target_pos, rel_target_orn, robot_state, reference_type)
            reference_type=ReferenceType.RELATIVE

            # pos = robot_state["tcp_pos"]
            print("target_pos", np.linalg.norm(np.array(target_pos) - np.array(converted_pos)))
            # obs, _, _, _ = env.step(action)
            # env.render()
            # fps.step()


if __name__ == "__main__":
    main()