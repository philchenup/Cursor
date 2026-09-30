#!/usr/bin/env python3
"""Vertical/inclined seams: orthonormal torch frame, no travel flip on steep welds."""

from __future__ import annotations

import math
import unittest

import numpy as np

EPS = 1e-12
WORLD_Z = np.array([0.0, 0.0, 1.0])
PREFERRED_TORCH = np.array([0.0, 0.0, -1.0])


class PointNormal:
    __slots__ = ("x", "y", "z", "normal_x", "normal_y", "normal_z")

    def __init__(self, xyz: np.ndarray, normal: np.ndarray) -> None:
        self.x, self.y, self.z = (float(v) for v in xyz)
        self.normal_x, self.normal_y, self.normal_z = (float(v) for v in normal)


class PointCloudPointNormal:
    def __init__(self, points: list[PointNormal] | None = None) -> None:
        self.points = list(points or [])

    def __len__(self) -> int:
        return len(self.points)


class Options:
    def __init__(
        self,
        world_up=WORLD_Z,
        steep_seam_abs_cos=0.5,
        preferred_torch=PREFERRED_TORCH,
        max_torch_tilt_deg=25.0,
        flip_travel_to_raise_y=True,
    ) -> None:
        self.world_up = np.asarray(world_up, dtype=np.float64)
        self.steep_seam_abs_cos = steep_seam_abs_cos
        self.preferred_torch = np.asarray(preferred_torch, dtype=np.float64)
        self.max_torch_tilt_deg = max_torch_tilt_deg
        self.flip_travel_to_raise_y = flip_travel_to_raise_y


def finite_unit(v: np.ndarray, fallback: np.ndarray) -> np.ndarray:
    if not np.isfinite(v[0]) or float(np.dot(v, v)) < EPS:
        return fallback / np.linalg.norm(fallback)
    return v / np.linalg.norm(v)


def project_perp(v: np.ndarray, axis: np.ndarray) -> np.ndarray:
    return v - axis * np.dot(v, axis)


def tilt_torch_toward(n: np.ndarray, preferred: np.ndarray, max_deg: float) -> np.ndarray:
    if float(np.dot(preferred, preferred)) < EPS or max_deg <= 0.0:
        return n
    p = preferred / np.linalg.norm(preferred)
    c = float(np.clip(np.dot(n, p), -1.0, 1.0))
    if c < 0.0:
        return n
    ang = math.acos(c)
    max_rad = math.radians(max_deg)
    if ang <= max_rad:
        return n
    axis = np.cross(n, p)
    if float(np.dot(axis, axis)) < EPS:
        return n
    axis = axis / np.linalg.norm(axis)
    K = np.array(
        [[0.0, -axis[2], axis[1]], [axis[2], 0.0, -axis[0]], [-axis[1], axis[0], 0.0]]
    )
    return n + math.sin(max_rad) * (K @ n) + (1.0 - math.cos(max_rad)) * (K @ (K @ n))


def travel_axis_in_torch_plane(z: np.ndarray, travel: np.ndarray, world_up: np.ndarray) -> np.ndarray:
    x = project_perp(travel, z)
    if float(np.dot(x, x)) < EPS:
        x = project_perp(world_up, z)
        if float(np.dot(x, x)) < EPS:
            alt = np.array([1.0, 0.0, 0.0]) if abs(z[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
            x = project_perp(alt, z)
    x = finite_unit(x, np.array([1.0, 0.0, 0.0]))
    if float(np.dot(x, travel)) < 0.0:
        x = -x
    return x


def compute_two_point_poses(trajectory: PointCloudPointNormal, opt: Options | None = None):
    opt = opt or Options()
    if len(trajectory) < 2:
        return False, None, None
    p0, p1 = trajectory.points[0], trajectory.points[1]
    t0 = np.array([p0.x, p0.y, p0.z], dtype=np.float64)
    t1 = np.array([p1.x, p1.y, p1.z], dtype=np.float64)
    weld = t1 - t0
    if float(np.dot(weld, weld)) < EPS:
        return False, None, None

    travel = weld / np.linalg.norm(weld)
    world_up = finite_unit(opt.world_up, WORLD_Z)
    steep = abs(float(np.dot(travel, world_up))) >= opt.steep_seam_abs_cos

    def prepare_z(q: PointNormal) -> np.ndarray:
        n = np.array([q.normal_x, q.normal_y, q.normal_z], dtype=np.float64)
        n = finite_unit(n, world_up)
        return tilt_torch_toward(n, opt.preferred_torch, opt.max_torch_tilt_deg)

    z0, z1 = prepare_z(p0), prepare_z(p1)
    z_mean = finite_unit(z0 + z1, z0)
    y_probe = np.cross(z_mean, travel)
    flip_xy = (
        opt.flip_travel_to_raise_y
        and not steep
        and float(np.dot(y_probe, y_probe)) >= EPS
        and float(np.dot(y_probe, world_up)) < 0.0
    )

    def make_pose(t: np.ndarray, z_in: np.ndarray) -> np.ndarray:
        z = finite_unit(z_in, world_up)
        x = travel_axis_in_torch_plane(z, travel, world_up)
        y = np.cross(z, x)
        if float(np.dot(y, y)) < EPS:
            y = np.cross(world_up, x)
        y = finite_unit(y, np.array([0.0, 1.0, 0.0]))
        if flip_xy:
            x, y = -x, -y
        z_rh = finite_unit(np.cross(x, y), z)
        T = np.eye(4, dtype=np.float64)
        T[:3, 0] = x
        T[:3, 1] = y
        T[:3, 2] = z_rh
        T[:3, 3] = t
        return T

    return True, make_pose(t0, z0), make_pose(t1, z1)


class WeldTorchFrameTests(unittest.TestCase):
    def assert_orthonormal_rh(self, R: np.ndarray) -> None:
        np.testing.assert_allclose(R.T @ R, np.eye(3), atol=1e-9)
        self.assertGreater(float(np.linalg.det(R)), 0.0)
        np.testing.assert_allclose(R[:, 1], np.cross(R[:, 2], R[:, 0]), atol=1e-9)

    def test_rejects_fewer_than_two_points(self) -> None:
        self.assertFalse(compute_two_point_poses(PointCloudPointNormal())[0])
        cloud = PointCloudPointNormal([PointNormal(np.zeros(3), WORLD_Z)])
        self.assertFalse(compute_two_point_poses(cloud)[0])

    def test_rejects_coincident_points(self) -> None:
        p = PointNormal(np.array([1.0, 2.0, 3.0]), WORLD_Z)
        self.assertFalse(compute_two_point_poses(PointCloudPointNormal([p, p]))[0])

    def test_inclined_floor_weld_projects_x_onto_torch_plane(self) -> None:
        """倾斜焊缝：X 必须 ⊥ 枪轴，否则旋转不正交，机械臂会拧。"""
        z = WORLD_Z
        t0 = np.zeros(3)
        weld = np.array([1.0, 0.2, 0.5])
        traj = PointCloudPointNormal([PointNormal(t0, z), PointNormal(t0 + weld, z)])
        ok, Ts, Te = compute_two_point_poses(traj, Options(max_torch_tilt_deg=0.0))
        self.assertTrue(ok)
        R = Ts[:3, :3]
        self.assert_orthonormal_rh(R)
        self.assertAlmostEqual(float(np.dot(R[:, 0], R[:, 2])), 0.0, places=6)
        projected = weld - z * np.dot(weld, z)
        projected = projected / np.linalg.norm(projected)
        self.assertGreater(float(np.dot(R[:, 0], projected)), 0.9)
        self.assert_orthonormal_rh(Te[:3, :3])

    def test_vertical_wall_keeps_upward_travel(self) -> None:
        """立焊缝：X 跟着起点→终点向上走，不因 Y·up≈0 而反向。"""
        n = np.array([0.0, 1.0, 0.0])
        t0 = np.array([0.0, 0.0, 0.0])
        t1 = np.array([0.0, 0.0, 2.0])
        traj = PointCloudPointNormal([PointNormal(t0, n), PointNormal(t1, n)])
        ok, Ts, Te = compute_two_point_poses(traj)
        self.assertTrue(ok)
        travel = t1 - t0
        self.assert_orthonormal_rh(Ts[:3, :3])
        self.assert_orthonormal_rh(Te[:3, :3])
        self.assertGreater(float(np.dot(Ts[:3, 0], travel)), 0.0)
        self.assertGreater(float(np.dot(Te[:3, 0], travel)), 0.0)
        self.assertGreater(float(np.dot(Ts[:3, 0], Te[:3, 0])), 0.0)
        self.assertLess(Ts[:3, 2][2], n[2])

    def test_steep_seam_does_not_reverse_travel_to_raise_y(self) -> None:
        n = np.array([1.0, 0.0, 0.0])
        t0 = np.zeros(3)
        t1 = np.array([0.1, 0.0, 1.0])
        traj = PointCloudPointNormal([PointNormal(t0, n), PointNormal(t1, n)])
        ok, Ts, _ = compute_two_point_poses(
            traj, Options(max_torch_tilt_deg=0.0, flip_travel_to_raise_y=True)
        )
        self.assertTrue(ok)
        self.assertGreater(float(np.dot(Ts[:3, 0], t1 - t0)), 0.0)
        self.assert_orthonormal_rh(Ts[:3, :3])

    def test_start_and_end_share_the_same_x_sign(self) -> None:
        t0 = np.array([0.0, 0.0, 1.0])
        t1 = np.array([0.4, 0.1, 0.9])
        n0 = np.array([0.0, 1.0, 0.2])
        n1 = np.array([0.1, 0.9, -0.2])
        traj = PointCloudPointNormal([PointNormal(t0, n0), PointNormal(t1, n1)])
        ok, Ts, Te = compute_two_point_poses(traj)
        self.assertTrue(ok)
        self.assertGreater(float(np.dot(Ts[:3, 0], Te[:3, 0])), 0.0)
        self.assert_orthonormal_rh(Ts[:3, :3])
        self.assert_orthonormal_rh(Te[:3, :3])

    def test_steep_shared_x_follows_start_to_end(self) -> None:
        t0 = np.array([0.0, 0.0, 0.0])
        t1 = np.array([0.2, 0.0, 1.0])
        n0 = np.array([0.0, 1.0, 0.2])
        n1 = np.array([0.1, 0.9, -0.1])
        traj = PointCloudPointNormal([PointNormal(t0, n0), PointNormal(t1, n1)])
        ok, Ts, Te = compute_two_point_poses(traj, Options(max_torch_tilt_deg=0.0))
        self.assertTrue(ok)
        weld = t1 - t0
        self.assertGreater(float(np.dot(Ts[:3, 0], Te[:3, 0])), 0.0)
        self.assertGreater(float(np.dot(Ts[:3, 0], weld)), 0.0)
        self.assertGreater(float(np.dot(Te[:3, 0], weld)), 0.0)

    def test_table_normal_opposite_preferred_is_not_tilted_through_part(self) -> None:
        t0 = np.zeros(3)
        t1 = np.array([1.0, 0.0, 0.0])
        traj = PointCloudPointNormal([PointNormal(t0, WORLD_Z), PointNormal(t1, WORLD_Z)])
        ok, Ts, _ = compute_two_point_poses(traj)
        self.assertTrue(ok)
        np.testing.assert_allclose(Ts[:3, 2], WORLD_Z, atol=1e-9)

    def test_invalid_normal_falls_back_to_world_up(self) -> None:
        t0 = np.zeros(3)
        t1 = np.array([1.0, 0.0, 0.0])
        traj = PointCloudPointNormal(
            [
                PointNormal(t0, np.array([math.nan, 0.0, 0.0])),
                PointNormal(t1, np.zeros(3)),
            ]
        )
        ok, Ts, Te = compute_two_point_poses(traj, Options(max_torch_tilt_deg=0.0))
        self.assertTrue(ok)
        np.testing.assert_allclose(Ts[:3, 2], WORLD_Z, atol=1e-9)
        np.testing.assert_allclose(Te[:3, 2], WORLD_Z, atol=1e-9)

    def test_flat_seam_may_flip_to_raise_y(self) -> None:
        z = np.array([0.0, 1.0, 0.0])
        t0 = np.zeros(3)
        weld = np.array([1.0, 0.0, 0.0])
        traj = PointCloudPointNormal([PointNormal(t0, z), PointNormal(t0 + weld, z)])
        ok, T, _ = compute_two_point_poses(traj, Options(max_torch_tilt_deg=0.0))
        self.assertTrue(ok)
        R = T[:3, :3]
        self.assert_orthonormal_rh(R)
        self.assertGreaterEqual(float(np.dot(R[:, 1], WORLD_Z)), -1e-12)
        np.testing.assert_allclose(R[:, 0], [-1.0, 0.0, 0.0], atol=1e-9)

    def test_travel_parallel_to_normal_still_orthonormal(self) -> None:
        weld = np.array([0.0, 0.0, 2.0])
        traj = PointCloudPointNormal(
            [PointNormal(np.zeros(3), WORLD_Z), PointNormal(weld, WORLD_Z)]
        )
        ok, T, _ = compute_two_point_poses(traj, Options(max_torch_tilt_deg=0.0))
        self.assertTrue(ok)
        self.assert_orthonormal_rh(T[:3, :3])
        self.assertTrue(np.all(np.isfinite(T)))
        self.assertAlmostEqual(float(np.dot(T[:3, 0], WORLD_Z)), 0.0, places=6)


if __name__ == "__main__":
    unittest.main()
