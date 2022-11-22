import copy

import numpy as np

from robot_io.actions.actions import Action


class BaseWorkSpace:
    """
    Base class for workspace.
    Clips an action or a position to ensure that robot stays within boundaries.
    Child classes implement different workspace shapes.
    """
    def is_inside(self, pos):
        """
        Check if a position is inside the workspace.

        Args:
            pos: The position to check.

        Returns:
            True if within workspace, False otherwise.
        """
        raise NotImplementedError

    def clip(self, x):
        """
        Clip a position or action to workspace.

        Args:
            x: Action or position.

        Returns:
            Clipped action or position.
        """
        x = copy.deepcopy(x)
        if isinstance(x, Action):
            if x.ref == "abs":
                x.target_pos = self.clip(x.target_pos)
            return x
        elif isinstance(x, (tuple, list, np.ndarray)) and len(x) == 3:
            return self._clip(x)
        else:
            raise ValueError

    def _clip(self, pos):
        raise NotImplementedError


class BoxWorkspace(BaseWorkSpace):
    """
    Box-shaped workspace.
    """
    def __init__(self, x_min, x_max, y_min, y_max, z_min, z_max):
        self.limits = np.array([[x_min, y_min, z_min], [x_max, y_max, z_max]])

    def is_inside(self, pos):
        return np.all(self.limits[0] <= pos) and np.all(pos <= self.limits[1])

    def _clip(self, pos):
        return np.clip(pos, self.limits[0], self.limits[1])

    def __str__(self):
        return f"Box workspace(x_min={self.limits[0,0]}, x_max={self.limits[1, 0]}, y_min={self.limits[0,1]}, y_max={self.limits[1,1]}, z_min={self.limits[0,2]}, z_max={self.limits[1,2]}"

class CylinderWorkspace(BaseWorkSpace):
    """
    Cylindrical workspace.
    """
    def __init__(self, r_min, r_max, z_min, z_max):
        self.r_min = r_min
        self.r_max = r_max
        self.z_min = z_min
        self.z_max = z_max

    def is_inside(self, pos):
        dist_center = np.sqrt(pos[0] ** 2 + pos[1] ** 2)
        return self.z_min <= dist_center <= self.z_max and self.r_min <= dist_center <= self.r_max

    def _clip(self, pos):
        if self.is_inside(pos):
            return pos.copy()
        clipped_pos = pos.copy()
        dist_center = np.sqrt(pos[0] ** 2 + pos[1] ** 2)
        if dist_center > self.r_max:
            theta = np.arctan2(pos[1], pos[0])
            clipped_pos[0] = np.cos(theta) * self.r_max
            clipped_pos[1] = np.sin(theta) * self.r_max
        elif dist_center < self.r_min:
            theta = np.arctan2(pos[1], pos[0])
            clipped_pos[0] = np.cos(theta) * self.r_min
            clipped_pos[1] = np.sin(theta) * self.r_min

        clipped_pos[2] = np.clip(pos[2], self.z_min, self.z_max)
        return clipped_pos

    def __str__(self):
        return f"Cylindrical workspace(r_min={self.r_min}, r_max={self.r_max}, z_min={self.z_min}, z_max={self.z_max}"

class HalfCylinderWorkspace(CylinderWorkspace):
    """
    Half-cylindrical workspace.
    """
    def __init__(self, r_min, r_max, z_min, z_max):
        super().__init__(r_min, r_max, z_min, z_max)

    def is_inside(self, pos):
        return pos[0] >= 0 and super().is_inside(pos)

    def _clip(self, pos):
        if self.is_inside(pos):
            return pos.copy()
        clipped_pos = pos.copy()
        if clipped_pos[0] <= 0:
            clipped_pos[0] = 0
        return super()._clip(clipped_pos)

    def __str__(self):
        return f"Half-cylindrical workspace(r_min={self.r_min}, r_max={self.r_max}, z_min={self.z_min}, z_max={self.z_max}"


