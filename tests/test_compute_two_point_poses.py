#!/usr/bin/env python3
"""Weld-parallel X + world-up Y contract for computeTwoPointPoses makePose."""

from __future__ import annotations

import unittest

import numpy as np

EPS = 1e-12
WORLD_Z = np.array([0.0, 0.0, 1.0])


def make_pose(t: np.ndarray, z_in: np.ndarray, weld_dir: np.ndarray) -> np.ndarray:
    """X // project(weld, ⊥Z); Y = Z × X, flip X and Y together if Y·world_Z < 0."""
    z = z_in / np.linalg.norm(z_in)
    x = weld_dir - z * np.dot(weld_dir, z)
    if np.dot(x, x) < EPS:
        axis = np.array([0.0, 0.0, 1.0]) if abs(z[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
        x = np.cross(axis, z)
    x = x / np.linalg.norm(x)
    y = np.cross(z, x)
    if np.dot(y, WORLD_Z) < 0.0:
        y = -y
        x = -x
    y = y / np.linalg.norm(y)
    T = np.eye(4, dtype=np.float64)
    T[:3, 0] = x
    T[:3, 1] = y
    T[:3, 2] = z
    T[:3, 3] = t
    return T


def weld_in_plane(weld_dir: np.ndarray, z: np.ndarray) -> np.ndarray | None:
    z = z / np.linalg.norm(z)
    x_proj = weld_dir - z * np.dot(weld_dir, z)
    n2 = float(np.dot(x_proj, x_proj))
    if n2 < EPS:
        return None
    return x_proj / np.sqrt(n2)


class WeldParallelWorldUpTests(unittest.TestCase):
    def assert_right_handed(self, R: np.ndarray) -> None:
        np.testing.assert_allclose(R.T @ R, np.eye(3), atol=1e-12)
        self.assertAlmostEqual(float(np.linalg.det(R)), 1.0, places=12)
        np.testing.assert_allclose(np.cross(R[:, 0], R[:, 1]), R[:, 2], atol=1e-12)
        np.testing.assert_allclose(R[:, 1], np.cross(R[:, 2], R[:, 0]), atol=1e-12)

    def assert_y_world_up(self, R: np.ndarray) -> None:
        self.assertGreaterEqual(float(np.dot(R[:, 1], WORLD_Z)), -1e-12)

    def assert_x_parallel_to_weld(self, R: np.ndarray, weld_dir: np.ndarray, z: np.ndarray) -> None:
        chord = weld_in_plane(weld_dir, z)
        if chord is None:
            return
        self.assertAlmostEqual(abs(float(np.dot(R[:, 0], chord))), 1.0, places=6)
        # X、焊缝、Z 共面：标量三重积为 0
        z_hat = z / np.linalg.norm(z)
        self.assertAlmostEqual(abs(float(np.dot(R[:, 0], np.cross(weld_dir, z_hat)))), 0.0, places=6)

    def test_x_parallel_to_start_end_when_weld_perp_z(self) -> None:
        weld = np.array([0.4, 0.0, 0.0])
        z = np.array([0.0, 0.0, 1.0])
        T = make_pose(np.zeros(3), z, weld)
        R = T[:3, :3]
        self.assert_right_handed(R)
        self.assert_x_parallel_to_weld(R, weld, z)
        self.assertAlmostEqual(abs(float(np.dot(R[:, 0], weld / np.linalg.norm(weld)))), 1.0)

    def test_x_stays_parallel_when_y_flips(self) -> None:
        z = np.array([0.0, 1.0, 0.0])
        weld = np.array([1.0, 0.0, 0.0])
        T = make_pose(np.zeros(3), z, weld)
        R = T[:3, :3]
        self.assert_right_handed(R)
        self.assert_y_world_up(R)
        self.assert_x_parallel_to_weld(R, weld, z)
        np.testing.assert_allclose(R[:, 0], [-1.0, 0.0, 0.0], atol=1e-12)

    def test_opposite_weld_still_parallel(self) -> None:
        z = np.array([0.0, 1.0, 0.0])
        weld = np.array([1.0, 0.0, 0.0])
        Tf = make_pose(np.zeros(3), z, weld)
        Tb = make_pose(np.zeros(3), z, -weld)
        self.assert_x_parallel_to_weld(Tf[:3, :3], weld, z)
        self.assert_x_parallel_to_weld(Tb[:3, :3], weld, z)
        np.testing.assert_allclose(np.abs(Tf[:3, 0]), np.abs(Tb[:3, 0]), atol=1e-12)

    def test_weld_with_z_component_x_follows_inplane_chord(self) -> None:
        z = np.array([0.0, 0.0, 1.0])
        weld = np.array([1.0, 0.2, 0.5])
        T = make_pose(np.zeros(3), z, weld)
        R = T[:3, :3]
        self.assert_right_handed(R)
        self.assert_x_parallel_to_weld(R, weld, z)
        inplane = np.array([1.0, 0.2, 0.0])
        inplane = inplane / np.linalg.norm(inplane)
        self.assertAlmostEqual(abs(float(np.dot(R[:, 0], inplane))), 1.0, places=6)

    def test_y_world_up_random_tilted(self) -> None:
        rng = np.random.default_rng(1)
        for _ in range(40):
            z = rng.normal(size=3)
            z[2] += 0.2
            weld = rng.normal(size=3)
            T = make_pose(np.zeros(3), z, weld)
            R = T[:3, :3]
            self.assert_right_handed(R)
            self.assert_y_world_up(R)
            self.assert_x_parallel_to_weld(R, weld, z)
            angle = float(np.degrees(np.arccos(np.clip(np.dot(R[:, 1], WORLD_Z), -1.0, 1.0))))
            self.assertLessEqual(angle, 90.0 + 1e-6)

    def test_two_poses_share_weld_dir(self) -> None:
        t0 = np.array([0.0, 0.0, 1.0])
        t1 = np.array([0.4, 0.1, 0.9])
        weld = t1 - t0
        z0 = np.array([0.0, 1.0, 0.2])
        z1 = np.array([0.1, 0.9, -0.2])
        Ts = make_pose(t0, z0, weld)
        Te = make_pose(t1, z1, weld)
        for T, z in ((Ts, z0), (Te, z1)):
            R = T[:3, :3]
            self.assert_right_handed(R)
            self.assert_y_world_up(R)
            self.assert_x_parallel_to_weld(R, weld, z)
        np.testing.assert_allclose(Ts[:3, 3], t0)
        np.testing.assert_allclose(Te[:3, 3], t1)

    def test_parallel_weld_and_z_fallback(self) -> None:
        T = make_pose(
            np.zeros(3),
            np.array([0.0, 0.0, 1.0]),
            np.array([0.0, 0.0, 2.0]),
        )
        R = T[:3, :3]
        self.assertTrue(np.all(np.isfinite(R)))
        self.assert_right_handed(R)
        self.assert_y_world_up(R)


if __name__ == "__main__":
    unittest.main()
