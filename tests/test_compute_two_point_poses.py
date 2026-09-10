#!/usr/bin/env python3
"""X is the 3D start→end vector (not projected onto ⊥Z). Y toward world +Z."""

from __future__ import annotations

import unittest

import numpy as np

EPS = 1e-12
WORLD_Z = np.array([0.0, 0.0, 1.0])


def make_pose(t: np.ndarray, z_in: np.ndarray, weld_dir: np.ndarray) -> np.ndarray:
    z = z_in / np.linalg.norm(z_in)
    x = weld_dir / np.linalg.norm(weld_dir)
    y = np.cross(z, x)
    if np.dot(y, y) < EPS:
        axis = np.array([0.0, 0.0, 1.0]) if abs(z[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
        y = np.cross(z, axis)
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


class StartEndXWorldUpYTests(unittest.TestCase):
    def assert_y_world_up(self, R: np.ndarray) -> None:
        self.assertGreaterEqual(float(np.dot(R[:, 1], WORLD_Z)), -1e-12)

    def assert_x_is_start_end(self, R: np.ndarray, weld_dir: np.ndarray) -> None:
        weld_hat = weld_dir / np.linalg.norm(weld_dir)
        self.assertAlmostEqual(abs(float(np.dot(R[:, 0], weld_hat))), 1.0, places=6)
        np.testing.assert_allclose(np.abs(R[:, 0]), np.abs(weld_hat), atol=1e-12)

    def test_x_equals_weld_even_when_weld_has_z_component(self) -> None:
        z = np.array([0.0, 0.0, 1.0])
        weld = np.array([1.0, 0.2, 0.5])
        T = make_pose(np.zeros(3), z, weld)
        R = T[:3, :3]
        weld_hat = weld / np.linalg.norm(weld)
        inplane = np.array([1.0, 0.2, 0.0])
        inplane = inplane / np.linalg.norm(inplane)
        self.assert_x_is_start_end(R, weld)
        self.assertLess(abs(float(np.dot(R[:, 0], inplane))), 0.99)
        np.testing.assert_allclose(np.abs(R[:, 0]), np.abs(weld_hat), atol=1e-12)
        self.assert_y_world_up(R)
        np.testing.assert_allclose(R[:, 2], z / np.linalg.norm(z))

    def test_x_is_not_the_inplane_projection(self) -> None:
        z = np.array([0.1, 0.2, 1.0])
        weld = np.array([0.4, 0.1, 0.3])
        T = make_pose(np.zeros(3), z, weld)
        R = T[:3, :3]
        z_hat = z / np.linalg.norm(z)
        projected = weld - z_hat * np.dot(weld, z_hat)
        projected = projected / np.linalg.norm(projected)
        self.assert_x_is_start_end(R, weld)
        self.assertLess(abs(float(np.dot(R[:, 0], projected))), 0.999)

    def test_y_flip_keeps_x_on_start_end_line(self) -> None:
        z = np.array([0.0, 1.0, 0.0])
        weld = np.array([1.0, 0.0, 0.0])
        T = make_pose(np.zeros(3), z, weld)
        R = T[:3, :3]
        self.assert_y_world_up(R)
        self.assert_x_is_start_end(R, weld)
        np.testing.assert_allclose(R[:, 0], [-1.0, 0.0, 0.0], atol=1e-12)
        np.testing.assert_allclose(R[:, 1], np.cross(R[:, 2], R[:, 0]), atol=1e-12)

    def test_y_world_up_random(self) -> None:
        rng = np.random.default_rng(2)
        for _ in range(40):
            z = rng.normal(size=3)
            z[2] += 0.2
            weld = rng.normal(size=3)
            if np.dot(weld, weld) < 1e-8:
                continue
            T = make_pose(np.zeros(3), z, weld)
            R = T[:3, :3]
            self.assert_x_is_start_end(R, weld)
            self.assert_y_world_up(R)
            np.testing.assert_allclose(R[:, 2], z / np.linalg.norm(z), atol=1e-12)
            angle = float(np.degrees(np.arccos(np.clip(np.dot(R[:, 1], WORLD_Z), -1.0, 1.0))))
            self.assertLessEqual(angle, 90.0 + 1e-6)

    def test_start_and_end_share_the_same_x_line(self) -> None:
        t0 = np.array([0.0, 0.0, 1.0])
        t1 = np.array([0.4, 0.1, 0.9])
        weld = t1 - t0
        Ts = make_pose(t0, np.array([0.0, 1.0, 0.2]), weld)
        Te = make_pose(t1, np.array([0.1, 0.9, -0.2]), weld)
        self.assert_x_is_start_end(Ts[:3, :3], weld)
        self.assert_x_is_start_end(Te[:3, :3], weld)
        self.assert_y_world_up(Ts[:3, :3])
        self.assert_y_world_up(Te[:3, :3])
        np.testing.assert_allclose(np.abs(Ts[:3, 0]), np.abs(weld / np.linalg.norm(weld)), atol=1e-12)
        np.testing.assert_allclose(np.abs(Te[:3, 0]), np.abs(weld / np.linalg.norm(weld)), atol=1e-12)

    def test_weld_parallel_to_z_keeps_x_on_weld(self) -> None:
        weld = np.array([0.0, 0.0, 2.0])
        T = make_pose(np.zeros(3), np.array([0.0, 0.0, 1.0]), weld)
        R = T[:3, :3]
        self.assert_x_is_start_end(R, weld)
        self.assert_y_world_up(R)
        self.assertTrue(np.all(np.isfinite(R)))


if __name__ == "__main__":
    unittest.main()
