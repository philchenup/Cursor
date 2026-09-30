#!/usr/bin/env python3
"""computeTwoPointPoses: trajectory[0]/[1] + PointNormal as Z; X = start→end."""

from __future__ import annotations

import math
import unittest

import numpy as np

EPS = 1e-12
WORLD_Z = np.array([0.0, 0.0, 1.0])


class PointNormal:
    __slots__ = ("x", "y", "z", "normal_x", "normal_y", "normal_z")

    def __init__(
        self,
        xyz: np.ndarray,
        normal: np.ndarray,
    ) -> None:
        self.x, self.y, self.z = (float(v) for v in xyz)
        self.normal_x, self.normal_y, self.normal_z = (float(v) for v in normal)


class PointCloudPointNormal:
    def __init__(self, points: list[PointNormal] | None = None) -> None:
        self.points = list(points or [])

    def size(self) -> int:
        return len(self.points)

    def __len__(self) -> int:
        return len(self.points)


def axis_z(q: PointNormal) -> np.ndarray:
    n = np.array([q.normal_x, q.normal_y, q.normal_z], dtype=np.float64)
    if not np.isfinite(n[0]) or float(np.dot(n, n)) < EPS:
        n = np.array([0.0, 0.0, 1.0])
    return n / np.linalg.norm(n)


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


def compute_two_point_poses(trajectory: PointCloudPointNormal):
    if len(trajectory) < 2:
        return False, None, None
    p0 = trajectory.points[0]
    p1 = trajectory.points[1]
    t0 = np.array([p0.x, p0.y, p0.z], dtype=np.float64)
    t1 = np.array([p1.x, p1.y, p1.z], dtype=np.float64)
    weld_dir = t1 - t0
    if float(np.dot(weld_dir, weld_dir)) < EPS:
        return False, None, None
    pose_start = make_pose(t0, axis_z(p0), weld_dir)
    pose_end = make_pose(t1, axis_z(p1), weld_dir)
    return True, pose_start, pose_end


class TrajectoryPointNormalTests(unittest.TestCase):
    def assert_y_world_up(self, R: np.ndarray) -> None:
        self.assertGreaterEqual(float(np.dot(R[:, 1], WORLD_Z)), -1e-12)

    def assert_x_is_start_end(self, R: np.ndarray, weld_dir: np.ndarray) -> None:
        weld_hat = weld_dir / np.linalg.norm(weld_dir)
        self.assertAlmostEqual(abs(float(np.dot(R[:, 0], weld_hat))), 1.0, places=6)
        np.testing.assert_allclose(np.abs(R[:, 0]), np.abs(weld_hat), atol=1e-12)

    def test_rejects_fewer_than_two_points(self) -> None:
        ok, *_ = compute_two_point_poses(PointCloudPointNormal())
        self.assertFalse(ok)
        cloud = PointCloudPointNormal([PointNormal(np.zeros(3), WORLD_Z)])
        ok, *_ = compute_two_point_poses(cloud)
        self.assertFalse(ok)

    def test_rejects_coincident_points(self) -> None:
        p = PointNormal(np.array([1.0, 2.0, 3.0]), WORLD_Z)
        ok, *_ = compute_two_point_poses(PointCloudPointNormal([p, p]))
        self.assertFalse(ok)

    def test_uses_each_point_normal_as_z(self) -> None:
        t0 = np.array([0.0, 0.0, 1.0])
        t1 = np.array([0.4, 0.1, 0.9])
        n0 = np.array([0.0, 1.0, 0.2])
        n1 = np.array([0.1, 0.9, -0.2])
        traj = PointCloudPointNormal(
            [PointNormal(t0, n0), PointNormal(t1, n1)]
        )
        ok, Ts, Te = compute_two_point_poses(traj)
        self.assertTrue(ok)
        weld = t1 - t0
        self.assert_x_is_start_end(Ts[:3, :3], weld)
        self.assert_x_is_start_end(Te[:3, :3], weld)
        np.testing.assert_allclose(Ts[:3, 2], n0 / np.linalg.norm(n0), atol=1e-12)
        np.testing.assert_allclose(Te[:3, 2], n1 / np.linalg.norm(n1), atol=1e-12)
        np.testing.assert_allclose(Ts[:3, 3], t0)
        np.testing.assert_allclose(Te[:3, 3], t1)
        self.assert_y_world_up(Ts[:3, :3])
        self.assert_y_world_up(Te[:3, :3])

    def test_invalid_normal_falls_back_to_unit_z(self) -> None:
        t0 = np.zeros(3)
        t1 = np.array([1.0, 0.0, 0.0])
        traj = PointCloudPointNormal(
            [
                PointNormal(t0, np.array([math.nan, 0.0, 0.0])),
                PointNormal(t1, np.zeros(3)),
            ]
        )
        ok, Ts, Te = compute_two_point_poses(traj)
        self.assertTrue(ok)
        np.testing.assert_allclose(Ts[:3, 2], WORLD_Z)
        np.testing.assert_allclose(Te[:3, 2], WORLD_Z)

    def test_x_equals_weld_even_when_weld_has_z_component(self) -> None:
        z = np.array([0.0, 0.0, 1.0])
        t0 = np.zeros(3)
        weld = np.array([1.0, 0.2, 0.5])
        traj = PointCloudPointNormal(
            [PointNormal(t0, z), PointNormal(t0 + weld, z)]
        )
        ok, T, _ = compute_two_point_poses(traj)
        self.assertTrue(ok)
        R = T[:3, :3]
        weld_hat = weld / np.linalg.norm(weld)
        inplane = np.array([1.0, 0.2, 0.0])
        inplane = inplane / np.linalg.norm(inplane)
        self.assert_x_is_start_end(R, weld)
        self.assertLess(abs(float(np.dot(R[:, 0], inplane))), 0.99)
        np.testing.assert_allclose(np.abs(R[:, 0]), np.abs(weld_hat), atol=1e-12)
        self.assert_y_world_up(R)
        np.testing.assert_allclose(R[:, 2], z / np.linalg.norm(z))

    def test_y_flip_keeps_x_on_start_end_line(self) -> None:
        z = np.array([0.0, 1.0, 0.0])
        t0 = np.zeros(3)
        weld = np.array([1.0, 0.0, 0.0])
        traj = PointCloudPointNormal(
            [PointNormal(t0, z), PointNormal(t0 + weld, z)]
        )
        ok, T, _ = compute_two_point_poses(traj)
        self.assertTrue(ok)
        R = T[:3, :3]
        self.assert_y_world_up(R)
        self.assert_x_is_start_end(R, weld)
        np.testing.assert_allclose(R[:, 0], [-1.0, 0.0, 0.0], atol=1e-12)
        np.testing.assert_allclose(R[:, 1], np.cross(R[:, 2], R[:, 0]), atol=1e-12)

    def test_weld_parallel_to_z_keeps_x_on_weld(self) -> None:
        weld = np.array([0.0, 0.0, 2.0])
        traj = PointCloudPointNormal(
            [
                PointNormal(np.zeros(3), WORLD_Z),
                PointNormal(weld, WORLD_Z),
            ]
        )
        ok, T, _ = compute_two_point_poses(traj)
        self.assertTrue(ok)
        R = T[:3, :3]
        self.assert_x_is_start_end(R, weld)
        self.assert_y_world_up(R)
        self.assertTrue(np.all(np.isfinite(R)))


if __name__ == "__main__":
    unittest.main()
