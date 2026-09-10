#!/usr/bin/env python3
"""Right-hand contract for computeTwoPointPoses makePose (mirrors the C++ header).

Old construction: Y = X × Z, then X = Z × Y  →  X × Y = −Z, det(R) = −1.
New construction: Y = Z × X, then X = Y × Z  →  X × Y = +Z, det(R) = +1.
"""

from __future__ import annotations

import unittest

import numpy as np

EPS = 1e-12


def make_pose_left_handed(t: np.ndarray, z_in: np.ndarray, x_in: np.ndarray) -> np.ndarray:
    """The original makePose: y = x_in × z, x = z × y."""
    z = z_in / np.linalg.norm(z_in)
    y = np.cross(x_in, z)
    if np.dot(y, y) < EPS:
        axis = np.array([0.0, 0.0, 1.0]) if abs(z[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
        y = np.cross(axis, z)
    y = y / np.linalg.norm(y)
    x = np.cross(z, y)
    x = x / np.linalg.norm(x)
    T = np.eye(4, dtype=np.float64)
    T[:3, 0] = x
    T[:3, 1] = y
    T[:3, 2] = z
    T[:3, 3] = t
    return T


def make_pose(t: np.ndarray, z_in: np.ndarray, x_in: np.ndarray) -> np.ndarray:
    """Right-handed: Y = Z × X_in, X = Y × Z so that X × Y = Z."""
    z = z_in / np.linalg.norm(z_in)
    y = np.cross(z, x_in)
    if np.dot(y, y) < EPS:
        axis = np.array([0.0, 0.0, 1.0]) if abs(z[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
        y = np.cross(z, axis)
    y = y / np.linalg.norm(y)
    x = np.cross(y, z)
    x = x / np.linalg.norm(x)
    T = np.eye(4, dtype=np.float64)
    T[:3, 0] = x
    T[:3, 1] = y
    T[:3, 2] = z
    T[:3, 3] = t
    return T


class RightHandPoseTests(unittest.TestCase):
    def test_old_construction_is_left_handed(self) -> None:
        T = make_pose_left_handed(
            np.array([0.0, 0.0, 0.0]),
            np.array([0.0, 0.0, 1.0]),
            np.array([1.0, 0.0, 0.0]),
        )
        R = T[:3, :3]
        self.assertAlmostEqual(float(np.linalg.det(R)), -1.0, places=12)
        np.testing.assert_allclose(np.cross(R[:, 0], R[:, 1]), -R[:, 2], atol=1e-12)

    def test_new_construction_is_right_handed(self) -> None:
        T = make_pose(
            np.array([1.0, 2.0, 3.0]),
            np.array([0.0, 0.0, 1.0]),
            np.array([1.0, 0.0, 0.0]),
        )
        R = T[:3, :3]
        np.testing.assert_allclose(R.T @ R, np.eye(3), atol=1e-12)
        self.assertAlmostEqual(float(np.linalg.det(R)), 1.0, places=12)
        np.testing.assert_allclose(np.cross(R[:, 0], R[:, 1]), R[:, 2], atol=1e-12)
        np.testing.assert_allclose(R[:, 1], np.cross(R[:, 2], R[:, 0]), atol=1e-12)
        np.testing.assert_allclose(R[:, 0], [1.0, 0.0, 0.0])
        np.testing.assert_allclose(R[:, 1], [0.0, 1.0, 0.0])
        np.testing.assert_allclose(R[:, 2], [0.0, 0.0, 1.0])
        np.testing.assert_allclose(T[:3, 3], [1.0, 2.0, 3.0])

    def test_y_equals_z_cross_x_not_x_cross_z(self) -> None:
        x_dir = np.array([0.2, 0.0, 0.0])
        z = np.array([0.0, 0.0, 1.0])
        T = make_pose(np.zeros(3), z, x_dir)
        y = T[:3, 1]
        np.testing.assert_allclose(y, np.cross(z, x_dir / np.linalg.norm(x_dir)), atol=1e-12)
        self.assertLess(float(np.dot(y, np.cross(x_dir, z))), 0.0)

    def test_x_follows_start_end_when_orthogonal_to_z(self) -> None:
        t0 = np.array([0.0, 0.0, 1.0])
        t1 = np.array([0.2, 0.0, 1.0])
        z = np.array([0.0, 0.0, 1.0])
        T = make_pose(t0, z, t1 - t0)
        self.assertGreater(float(np.dot(T[:3, 0], t1 - t0)), 0.0)
        self.assertLess(abs(float(np.dot(T[:3, 0], z))), 1e-12)
        self.assertLess(abs(float(np.dot(T[:3, 1], z))), 1e-12)

    def test_projects_x_off_z_and_stays_right_handed(self) -> None:
        z = np.array([0.0, 0.0, 1.0])
        x_in = np.array([1.0, 0.0, 1.0])
        T = make_pose(np.zeros(3), z, x_in)
        R = T[:3, :3]
        self.assertAlmostEqual(float(np.linalg.det(R)), 1.0, places=12)
        np.testing.assert_allclose(R[:, 2], z)
        self.assertGreater(float(np.dot(R[:, 0], np.array([1.0, 0.0, 0.0]))), 0.0)
        self.assertLess(abs(float(np.dot(R[:, 0], z))), 1e-12)
        np.testing.assert_allclose(R[:, 1], np.cross(R[:, 2], R[:, 0]), atol=1e-12)

    def test_parallel_x_and_z_uses_fallback_and_stays_right_handed(self) -> None:
        T = make_pose(
            np.zeros(3),
            np.array([0.0, 0.0, 1.0]),
            np.array([0.0, 0.0, 2.0]),
        )
        R = T[:3, :3]
        self.assertTrue(np.all(np.isfinite(R)))
        np.testing.assert_allclose(R.T @ R, np.eye(3), atol=1e-12)
        self.assertAlmostEqual(float(np.linalg.det(R)), 1.0, places=12)
        np.testing.assert_allclose(np.cross(R[:, 0], R[:, 1]), R[:, 2], atol=1e-12)

    def test_tilted_normal_right_hand_and_x_alignment(self) -> None:
        z = np.array([0.1, 0.2, 1.0])
        x_dir = np.array([1.0, 0.3, 0.0])
        T = make_pose(np.array([4.0, 5.0, 6.0]), z, x_dir)
        R = T[:3, :3]
        z_hat = z / np.linalg.norm(z)
        np.testing.assert_allclose(R[:, 2], z_hat, atol=1e-12)
        np.testing.assert_allclose(R.T @ R, np.eye(3), atol=1e-12)
        self.assertAlmostEqual(float(np.linalg.det(R)), 1.0, places=12)
        np.testing.assert_allclose(R[:, 1], np.cross(R[:, 2], R[:, 0]), atol=1e-12)
        self.assertGreater(float(np.dot(R[:, 0], x_dir)), 0.0)

    def test_two_poses_share_x_dir_and_are_right_handed(self) -> None:
        t0 = np.array([0.0, 0.0, 1.0])
        t1 = np.array([0.4, 0.1, 1.0])
        z0 = np.array([0.0, 0.0, 1.0])
        z1 = np.array([0.05, 0.0, 1.0])
        x_dir = t1 - t0
        Ts = make_pose(t0, z0, x_dir)
        Te = make_pose(t1, z1, x_dir)
        for T in (Ts, Te):
            R = T[:3, :3]
            self.assertAlmostEqual(float(np.linalg.det(R)), 1.0, places=12)
            np.testing.assert_allclose(np.cross(R[:, 0], R[:, 1]), R[:, 2], atol=1e-12)
            self.assertGreater(float(np.dot(R[:, 0], x_dir)), 0.0)
        np.testing.assert_allclose(Ts[:3, 3], t0)
        np.testing.assert_allclose(Te[:3, 3], t1)


if __name__ == "__main__":
    unittest.main()
