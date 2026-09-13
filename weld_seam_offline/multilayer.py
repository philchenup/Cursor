"""Two-pass V-groove offset used by demo_all.mutilayer()."""

from __future__ import annotations

import copy
import math

import numpy as np
from scipy.spatial.transform import Rotation as R


def offset_along_normal(
    poses: np.ndarray, offset_z: float = -0.003, offset_y: float = -0.002
) -> np.ndarray:
    """demo_all.uplift_z: shift TCP slightly along torch Y/Z."""
    rotation = R.from_rotvec(poses[0, 3:6])
    matrix = rotation.as_matrix()
    new_z = matrix[:, 2]
    new_y = matrix[:, 1]
    displacement = offset_z * new_z + offset_y * new_y
    out = np.array(poses, copy=True)
    out[:, :3] += displacement
    return out


def multilayer_passes(
    poses: np.ndarray, z_height: float = -0.004, y_height: float = -0.006
) -> tuple[np.ndarray, np.ndarray]:
    """Left/right fill passes around the root pass (demo_all.mutilayer)."""
    origin = R.from_rotvec(poses[0, 3:6])
    matrix = origin.as_matrix()
    z_offset = matrix[:, 2] * z_height
    y_offset = matrix[:, 1] * y_height
    angle = math.atan2(y_height, z_height)
    left_angle = angle / 2.0 - np.pi / 4.0

    left = copy.deepcopy(poses)
    right = copy.deepcopy(poses)
    left_rot = (R.from_euler("x", left_angle) * origin).as_rotvec()
    right_rot = (R.from_euler("x", -left_angle) * origin).as_rotvec()

    left[:, :3] += z_offset + y_offset
    right[:, :3] += z_offset - y_offset
    left[:, 3:6] = left_rot
    right[:, 3:6] = right_rot
    return left, right
