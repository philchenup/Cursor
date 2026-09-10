#!/usr/bin/env python3
"""Right-hand + world-up Y contract for computeTwoPointPoses makePose."""

from __future__ import annotations

import unittest

import numpy as np

EPS = 1e-12
WORLD_Z = np.array([0.0, 0.0, 1.0])


def make_pose(t: np.ndarray, z_in: np.ndarray, x_in: np.ndarray) -> np.ndarray:
    """Y = Z × X, flip Y so Y·world_Z ≥ 0, then X = Y × Z (parallel to weld)."""
    z = z_in / np.linalg.norm(z_in)
    y = np.cross(z, x_in)
    if np.dot(y, y) < EPS:
        axis = np.array([0.0, 0.0, 1.0]) if abs(z[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
        y = np.cross(z, axis)
    if np.dot(y, WORLD_Z) < 0.0:
        y = -y
    y = y / np.linalg.norm(y)
    x = np.cross(y, z)
    x = x / np.linalg.norm(x)
    T = np.eye(4, dtype=np.float64)
    T[:3, 0] = x
    T[:3, 1] = y
    T[:3, 2] = z
    T[:3, 3] = t
    return T


def chord_dir_in_plane(x_dir: np.ndarray, z: np.ndarray) -> np.ndarray | None:
    z = z / np.linalg.norm(z)
    x_proj = x_dir - z * np.dot(x_dir, z)
    n2 = float(np.dot(x_proj, x_proj))
    if n2 < EPS:
        return None
    return x_proj / np.sqrt(n2)


class WorldUpRightHandPoseTests(unittest.TestCase):
    def assert_right_handed(self, R: np.ndarray) -> None:
        np.testing.assert_allclose(R.T @ R, np.eye(3), atol=1e-12)
        self.assertAlmostEqual(float(np.linalg.det(R)), 1.0, places=12)
        np.testing.assert_allclose(np.cross(R[:, 0], R[:, 1]), R[:, 2], atol=1e-12)
        np.testing.assert_allclose(R[:, 1], np.cross(R[:, 2], R[:, 0]), atol=1e-12)

    def assert_y_world_up(self, R: np.ndarray) -> None:
        self.assertGreaterEqual(float(np.dot(R[:, 1], WORLD_Z)), -1e-12)

    def assert_x_parallel_to_weld(self, R: np.ndarray, x_dir: np.ndarray, z: np.ndarray) -> None:
        chord = chord_dir_in_plane(x_dir, z)
        if chord is None:
            return
        self.assertAlmostEqual(abs(float(np.dot(R[:, 0], chord))), 1.0, places=6)

    def test_horizontal_z_keeps_right_hand(self) -> None:
        T = make_pose(
            np.array([1.0, 2.0, 3.0]),
            np.array([0.0, 0.0, 1.0]),
            np.array([1.0, 0.0, 0.0]),
        )
        R = T[:3, :3]
        self.assert_right_handed(R)
        self.assert_y_world_up(R)
        np.testing.assert_allclose(R[:, 0], [1.0, 0.0, 0.0])
        np.testing.assert_allclose(R[:, 1], [0.0, 1.0, 0.0])
        np.testing.assert_allclose(R[:, 2], [0.0, 0.0, 1.0])

    def test_flips_y_to_world_up_and_x_stays_parallel(self) -> None:
        # Z = +world Y, chord = +world X → Z×X = −world Z, must flip Y to +world Z.
        z = np.array([0.0, 1.0, 0.0])
        x_dir = np.array([1.0, 0.0, 0.0])
        T = make_pose(np.zeros(3), z, x_dir)
        R = T[:3, :3]
        self.assert_right_handed(R)
        self.assert_y_world_up(R)
        np.testing.assert_allclose(R[:, 1], [0.0, 0.0, 1.0], atol=1e-12)
        self.assert_x_parallel_to_weld(R, x_dir, z)
        np.testing.assert_allclose(R[:, 0], [-1.0, 0.0, 0.0], atol=1e-12)

    def test_opposite_chord_still_y_world_up(self) -> None:
        z = np.array([0.0, 1.0, 0.0])
        x_fwd = np.array([1.0, 0.0, 0.0])
        Tf = make_pose(np.zeros(3), z, x_fwd)
        Tb = make_pose(np.zeros(3), z, -x_fwd)
        self.assert_y_world_up(Tf[:3, :3])
        self.assert_y_world_up(Tb[:3, :3])
        np.testing.assert_allclose(Tf[:3, 1], Tb[:3, 1], atol=1e-12)
        np.testing.assert_allclose(Tf[:3, 0], Tb[:3, 0], atol=1e-12)

    def test_y_dot_world_z_nonnegative_for_tilted_normals(self) -> None:
        rng = np.random.default_rng(0)
        for _ in range(40):
            z = rng.normal(size=3)
            z[2] += 0.2
            x_dir = rng.normal(size=3)
            T = make_pose(np.zeros(3), z, x_dir)
            R = T[:3, :3]
            self.assert_right_handed(R)
            self.assert_y_world_up(R)
            self.assert_x_parallel_to_weld(R, x_dir, z / np.linalg.norm(z))
            angle = float(np.degrees(np.arccos(np.clip(np.dot(R[:, 1], WORLD_Z), -1.0, 1.0))))
            self.assertLessEqual(angle, 90.0 + 1e-6)

    def test_x_not_required_to_match_start_to_end_sign(self) -> None:
        z = np.array([0.2, 0.8, 0.1])
        t0 = np.array([0.0, 0.0, 1.0])
        t1 = np.array([0.4, 0.1, 1.0])
        T = make_pose(t0, z, t1 - t0)
        R = T[:3, :3]
        self.assert_right_handed(R)
        self.assert_y_world_up(R)
        self.assert_x_parallel_to_weld(R, t1 - t0, z)

    def test_parallel_x_and_z_uses_fallback_world_up(self) -> None:
        T = make_pose(
            np.zeros(3),
            np.array([0.0, 0.0, 1.0]),
            np.array([0.0, 0.0, 2.0]),
        )
        R = T[:3, :3]
        self.assertTrue(np.all(np.isfinite(R)))
        self.assert_right_handed(R)
        self.assert_y_world_up(R)

    def test_two_poses_each_world_up_right_handed(self) -> None:
        t0 = np.array([0.0, 0.0, 1.0])
        t1 = np.array([0.4, 0.1, 1.0])
        z0 = np.array([0.0, 1.0, 0.2])
        z1 = np.array([0.1, 0.9, -0.2])
        x_dir = t1 - t0
        Ts = make_pose(t0, z0, x_dir)
        Te = make_pose(t1, z1, x_dir)
        for T, z in ((Ts, z0), (Te, z1)):
            R = T[:3, :3]
            self.assert_right_handed(R)
            self.assert_y_world_up(R)
            self.assert_x_parallel_to_weld(R, x_dir, z)
        np.testing.assert_allclose(Ts[:3, 3], t0)
        np.testing.assert_allclose(Te[:3, 3], t1)


if __name__ == "__main__":
    unittest.main()
