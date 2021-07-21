"""
SpaceMouse input class
"""
import time
import numpy as np

try:
    import spnav
except AttributeError as err:
    print(err)
    print("Try uncommenting lines 7 and 8 from spnav.__init__.py")
except OSError as err:
    print(err)
    print("Try to rename 'libspnav.so' to e.g. 'libspnav.so.0' in line 10 of spnav.__init__.py")


class SpaceMouse:
    """SpaceMouse input class"""
    def __init__(self, act_type='continuous', sensitivity=100, mode="5dof", initial_gripper_state='open', **kwargs):
        """

        :param act_type: ['continuous', 'discrete'] specifies action output
        :param sensitivity: lower value corresponds to higher sensitivity
        :param mode: ['5dof', '7dof'] how many DOF of the robot are controlled
        :param initial_gripper_state: ['open', 'closed'] initial opening of gripper
        """
        # Note: this hangs if the SpaceNav mouse is not connecting properly
        # mostly commenting out this imput stream will be ok.
        # TODO(max): clean this up with a timeout/conditional messages maybe
        print("Opening SpaceNav, connect/comment if hangs")
        spnav.spnav_open()
        print("Opening SpaceNav: done")
        assert mode in ['5dof', '7dof']
        assert initial_gripper_state in ['open', 'closed']
        self.mode = mode
        self._gripper_state = 1 if initial_gripper_state == 'open' else -1
        assert act_type in ['continuous', 'discrete']
        self.act_type = act_type
        self.threshold = sensitivity

    def __del__(self):
        spnav.spnav_close()

    def handle_mouse_events(self):
        """process events"""
        if self.mode == '5dof':
            return self.handle_mouse_events_5dof()
        if self.mode == '7dof':
            return self.handle_mouse_events_7dof()

        raise ValueError

    def handle_mouse_events_5dof(self):
        """5dof mode xyz + a + gripper"""
        def map_action(action):
            if abs(action) < self.threshold:
                return 0
            return np.sign(action) * ((abs(action) - self.threshold) / (500.0 - self.threshold))

        event = spnav.spnav_poll_event()
        if event is not None:
            if event.ev_type == spnav.SPNAV_EVENT_MOTION:
                x = -map_action(event.rotation[2])
                y = map_action(event.rotation[0])
                z = map_action(event.translation[1])
                rot_z = -map_action(event.rotation[1])
                return self._process_action_type([x, y, z, rot_z, self._gripper_state])
            if event.ev_type == spnav.SPNAV_EVENT_BUTTON:
                if event.bnum == 0 and event.press:
                    self._gripper_state = -1
                    print("close")
                elif event.bnum == 1 and event.press:
                    self._gripper_state = 1
                    print('open')
        return self._process_action_type([0, 0, 0, 0, self._gripper_state])

    def handle_mouse_events_7dof(self):
        """7 dof mode, xyz + abc + gripper"""
        def map_action(action):
            if abs(action) < self.threshold:
                return 0
            return np.sign(action) * ((abs(action) - self.threshold) / (500.0 - self.threshold))

        event = spnav.spnav_poll_event()
        if event is not None:
            if event.ev_type == spnav.SPNAV_EVENT_MOTION:
                x = -map_action(event.translation[0])
                y = -map_action(event.translation[2])
                z = map_action(event.translation[1])
                roll = map_action(event.rotation[0])
                yaw = -map_action(event.rotation[1])
                pitch = -map_action(event.rotation[2])
                return [x, y, z, roll, pitch, yaw, self._gripper_state]
            if event.ev_type == spnav.SPNAV_EVENT_BUTTON:
                if event.bnum == 0 and event.press:
                    self._gripper_state = -1
                    print("close")
                elif event.bnum == 1 and event.press:
                    self._gripper_state = 1
                    print('open')
        return [0, 0, 0, 0, 0, 0, self._gripper_state]

    def _process_action_type(self, action):
        if self.act_type == 'continuous':
            return action

        mdisc_a = [0] * len(action)
        for i, act in enumerate(action):
            if act < 0:
                mdisc_a[i] = 0
            elif act == 0:
                mdisc_a[i] = 1
            else:
                mdisc_a[i] = 2
        return mdisc_a

    @staticmethod
    def clear_events():
        '''clear events'''
        spnav.spnav_remove_events(spnav.SPNAV_EVENT_MOTION)


def test_mouse():
    """test mouse, print actions"""
    mouse = SpaceMouse(act_type='continuous')
    for i in range(int(1e6)):
        action = mouse.handle_mouse_events()
        print(i, action)
        mouse.clear_events()
        time.sleep(.02)


if __name__ == "__main__":
    test_mouse()
