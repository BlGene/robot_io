"""
SpaceMouse input class
"""
import time
import numpy as np

from robot_io.actions.actions import Action
from robot_io.input_devices.base_input_device import BaseInputDevice

try:
    import spnav
except AttributeError as err:
    print(err)
    print("Try uncommenting lines 7 and 8 from spnav.__init__.py")
except OSError as err:
    print(err)
    print("Try to rename 'libspnav.so' to e.g. 'libspnav.so.0' in line 10 of spnav.__init__.py")

GRIPPER_CLOSING_ACTION = -1
GRIPPER_OPENING_ACTION = 1


class SpaceMouse(BaseInputDevice):
    """
    SpaceMouse input class

    Args:
        sensitivity: Lower value corresponds to higher sensitivity
        mode: ['5dof', '7dof'] how many DOF of the robot are controlled
        initial_gripper_state: ['open', 'closed'] initial opening of gripper
        dv: Factor for cartesian position offset in relative cartesian position control, in meter
        drot: Factor for orientation offset of gripper rotation in relative cartesian position control, in radians
    """
    def __init__(self, action_params, sensitivity=100, mode="5dof", initial_gripper_state='open',
                 dv=0.01, drot=0.2, reference_frame='tcp', **kwargs):
        # Note: this hangs if the SpaceNav mouse is not connecting properly
        # mostly commenting out this imput stream will be ok.
        # TODO(max): clean this up with a timeout/conditional messages maybe
        print("Opening SpaceNav, connect/comment if hangs")
        spnav.spnav_open()
        print("Opening SpaceNav: done")
        assert mode in ['5dof', '7dof']
        assert initial_gripper_state in ['open', 'closed']
        self._action_params = action_params
        self.mode = mode
        self._gripper_state = 1 if initial_gripper_state == 'open' else -1
        self.threshold = sensitivity
        self.dv = dv
        self.drot = drot
        # Filter for random zeros of our current space mouse
        self.filter = True
        self.prev_pos = np.zeros(3)
        self.prev_orn = np.zeros(3)
        self.done = False

    def __del__(self):
        spnav.spnav_close()

    def get_action(self):
        """
        Get the action dictionary from 3d-mouse.

        Returns:
            action (dict): Keyboard action.
            record_info: None (to be consistent with other input devices).
        """
        if self.mode == '5dof':
            sm_action = self.handle_mouse_events_5dof()
            sm_action = np.array(sm_action)
            sm_controller_pos = sm_action[0:3] * self.dv
            sm_controller_orn = np.array([0, 0, sm_action[3] * self.drot])
            gripper_action = sm_action[4]

        elif self.mode == '7dof':
            sm_action = self.handle_mouse_events_7dof()
            sm_action = np.array(sm_action)
            sm_controller_pos = sm_action[0:3] * self.dv
            sm_controller_orn = np.array([sm_action[3] * self.drot, sm_action[4] * self.drot, sm_action[5] * self.drot])
            gripper_action = sm_action[6]

        else:
            raise ValueError

        if np.all((sm_controller_pos == 0)) and np.all((sm_controller_orn == 0)):
            if self.filter:
                sm_controller_pos = self.prev_pos
                sm_controller_orn = self.prev_orn
                self.filter = False
        else:
            self.prev_pos = sm_controller_pos
            self.prev_orn = sm_controller_orn
            self.filter = True

        action = Action(target_pos=sm_controller_pos,
                        target_orn=sm_controller_orn,
                        gripper_action=gripper_action,
                        ref=self._action_params.ref,
                        path=self._action_params.path,
                        blocking=False,
                        impedance=self._action_params.impedance)

        # To be compatible with vr input actions. For now there is nothing to pass as record info
        record_info = {"done": self.done}
        self.clear_events()

        return action, record_info

    def handle_mouse_events(self):
        """process events"""
        if self.mode == '5dof':
            return self.handle_mouse_events_5dof()
        if self.mode == '7dof':
            return self.handle_mouse_events_7dof()

        raise ValueError

    def handle_mouse_button(self, event):
        if event.bnum == 0 and event.press:
            if self._gripper_state == 1:
                self._gripper_state = -1
                print("close")
            elif self._gripper_state == -1:
                self._gripper_state = 1
                print('open')
            else:
                raise ValueError
        if event.bnum == 1 and event.press:
            self.done = True


    def handle_mouse_events_5dof(self):
        """5dof mode xyz + a + gripper"""
        def map_action(action):
            if abs(action) < self.threshold:
                return 0
            return np.sign(action) * ((abs(action) - self.threshold) / (500.0 - self.threshold))

        event = spnav.spnav_poll_event()
        if event is not None:
            if event.ev_type == spnav.SPNAV_EVENT_MOTION:
                x = -map_action(event.rotation[0])
                y = map_action(event.rotation[2])
                z = -map_action(event.translation[1])
                rot_z = -map_action(event.rotation[1])

                if self._action_params.ref == "rel_world":
                    y *= -1
                    z *= -1
                    rot_z *= -1
                return [x, y, z, rot_z, self._gripper_state]
            if event.ev_type == spnav.SPNAV_EVENT_BUTTON:
                self.handle_mouse_button(event)

        return [0, 0, 0, 0, self._gripper_state]

    def handle_mouse_events_7dof(self):
        """7 dof mode, xyz + abc + gripper"""
        def map_action(action):
            if abs(action) < self.threshold:
                return 0
            return np.sign(action) * ((abs(action) - self.threshold) / (500.0 - self.threshold))

        event = spnav.spnav_poll_event()
        if event is not None:
            if event.ev_type == spnav.SPNAV_EVENT_MOTION:
                x = map_action(event.translation[2])
                y = map_action(event.translation[0])
                z = -map_action(event.translation[1])
                roll = map_action(event.rotation[2])
                pitch = map_action(event.rotation[0])
                yaw = -map_action(event.rotation[1])
                return [x, y, z, roll, pitch, yaw, self._gripper_state]
            if event.ev_type == spnav.SPNAV_EVENT_BUTTON:
                if event.bnum == 0 and event.press:
                    self._gripper_state = -1
                    print("close")
                elif event.bnum == 1 and event.press:
                    self._gripper_state = 1
                    print('open')
        return [0, 0, 0, 0, 0, 0, self._gripper_state]

    @staticmethod
    def clear_events():
        spnav.spnav_remove_events(spnav.SPNAV_EVENT_MOTION)


def print_actions():
    """test mouse, print actions"""
    mouse = SpaceMouse(act_type='continuous', mode='7dof')
    for i in range(int(1e6)):
        action = mouse.get_action()
        print(i, action[0]['motion'])
        time.sleep(.02)


if __name__ == "__main__":
    print_actions()
