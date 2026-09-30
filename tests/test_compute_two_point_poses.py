#!/usr/bin/env python3
"""Flat PA: X=travel. Vertical PF: Y=bottom-to-top, X=horizontal."""

from __future__ import annotations

import math
import unittest

import numpy as np

EPS = 1e-12
WORLD_Z = np.array([0.0, 0.0, 1.0])
PREFERRED_TORCH = np.array([0.0, 0.0, -1.0])
KEEP, DOWNHILL, UPHILL = "keep", "downhill", "uphill"
AUTO, FLAT, VERTICAL = "auto", "flat", "vertical"


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
        travel_policy=DOWNHILL,
        weld_position=AUTO,
        vertical_seam_abs_cos=0.5,
        steep_seam_abs_cos=0.5,
        preferred_torch=PREFERRED_TORCH,
        max_torch_tilt_deg=25.0,
        travel_angle_deg=10.0,
        gravity_signed_travel_angle=True,
        tool_head_axis=None,
        flip_travel_to_raise_y=False,
    ) -> None:
        self.world_up = np.asarray(world_up, dtype=np.float64)
        self.travel_policy = travel_policy
        self.weld_position = weld_position
        self.vertical_seam_abs_cos = vertical_seam_abs_cos
        self.steep_seam_abs_cos = steep_seam_abs_cos
        self.preferred_torch = np.asarray(preferred_torch, dtype=np.float64)
        self.max_torch_tilt_deg = max_torch_tilt_deg
        self.travel_angle_deg = travel_angle_deg
        self.gravity_signed_travel_angle = gravity_signed_travel_angle
        self.tool_head_axis = np.asarray(
            [1.0, 0.0, 0.0] if tool_head_axis is None else tool_head_axis,
            dtype=np.float64,
        )
        self.flip_travel_to_raise_y = flip_travel_to_raise_y


def finite_unit(v: np.ndarray, fallback: np.ndarray) -> np.ndarray:
    if not np.isfinite(v[0]) or float(np.dot(v, v)) < EPS:
        return fallback / np.linalg.norm(fallback)
    return v / np.linalg.norm(v)


def project_perp(v: np.ndarray, axis: np.ndarray) -> np.ndarray:
    return v - axis * np.dot(v, axis)


def is_vertical_seam(seam: np.ndarray, world_up: np.ndarray, opt: Options) -> bool:
    if opt.weld_position == FLAT:
        return False
    if opt.weld_position == VERTICAL:
        return True
    return abs(float(np.dot(seam, world_up))) >= opt.vertical_seam_abs_cos


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
    t = project_perp(travel, z)
    if float(np.dot(t, t)) < EPS:
        t = project_perp(world_up, z)
        if float(np.dot(t, t)) < EPS:
            alt = np.array([1.0, 0.0, 0.0]) if abs(z[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
            t = project_perp(alt, z)
    t = finite_unit(t, np.array([1.0, 0.0, 0.0]))
    if float(np.dot(t, travel)) < 0.0:
        t = -t
    return t


def make_frame(x: np.ndarray, y: np.ndarray, z: np.ndarray):
    x = finite_unit(x, np.array([1.0, 0.0, 0.0]))
    y = finite_unit(y, np.array([0.0, 1.0, 0.0]))
    z = finite_unit(np.cross(x, y), z)
    y = finite_unit(np.cross(z, x), y)
    return x, y, z


def rotate_around_y(x, y, z, rad):
    c, s = math.cos(rad), math.sin(rad)
    return make_frame(x * c + z * s, y, -x * s + z * c)


def rotate_around_x(x, y, z, rad):
    c, s = math.cos(rad), math.sin(rad)
    return make_frame(x, y * c + z * s, -y * s + z * c)


def rotate_travel_angle(x, y, z, rad, vertical):
    return rotate_around_x(x, y, z, rad) if vertical else rotate_around_y(x, y, z, rad)


def head_up(x, y, z, tool_head, world_up) -> float:
    R = np.column_stack((x, y, z))
    head = R @ finite_unit(tool_head, np.array([1.0, 0.0, 0.0]))
    return float(np.dot(head, world_up))


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

    world_up = finite_unit(opt.world_up, WORLD_Z)
    seam = weld / np.linalg.norm(weld)
    vertical = is_vertical_seam(seam, world_up, opt)

    def prepare_z(q: PointNormal) -> np.ndarray:
        n = np.array([q.normal_x, q.normal_y, q.normal_z], dtype=np.float64)
        n = finite_unit(n, world_up)
        return tilt_torch_toward(n, opt.preferred_torch, opt.max_torch_tilt_deg)

    z0, z1 = prepare_z(p0), prepare_z(p1)
    policy = opt.travel_policy
    if vertical and policy != KEEP:
        policy = UPHILL
    travel = seam.copy()
    du = float(np.dot(travel, world_up))
    swap = (policy == DOWNHILL and du > 1e-6) or (policy == UPHILL and du < -1e-6)
    if swap:
        t0, t1 = t1.copy(), t0.copy()
        z0, z1 = z1.copy(), z0.copy()
        travel = -travel

    steep = abs(float(np.dot(travel, world_up))) >= opt.steep_seam_abs_cos
    z_mean = finite_unit(z0 + z1, z0)
    y_probe = np.cross(z_mean, travel)
    flip_xy = (
        (not vertical)
        and opt.flip_travel_to_raise_y
        and not steep
        and float(np.dot(y_probe, y_probe)) >= EPS
        and float(np.dot(y_probe, world_up)) < 0.0
    )

    def assemble(z_in):
        z = finite_unit(z_in, world_up)
        along = travel_axis_in_torch_plane(z, travel, world_up)
        if vertical:
            y = along
            x = np.cross(y, z)
            if float(np.dot(x, x)) < EPS:
                x = np.cross(world_up, z)
            return make_frame(x, y, z)
        x = along
        y = np.cross(z, x)
        if float(np.dot(y, y)) < EPS:
            y = np.cross(world_up, x)
        if flip_xy:
            x, y = -x, -y
        return make_frame(x, y, z)

    px, py, pz = assemble(z_mean)
    mag = abs(opt.travel_angle_deg) * math.pi / 180.0
    signed = 0.0
    if mag > 1e-8:
        plus = mag if opt.travel_angle_deg >= 0.0 else -mag
        if not opt.gravity_signed_travel_angle:
            signed = plus
        else:
            ax, ay, az = rotate_travel_angle(px, py, pz, plus, vertical)
            bx, by, bz = rotate_travel_angle(px, py, pz, -plus, vertical)
            signed = (
                plus
                if head_up(ax, ay, az, opt.tool_head_axis, world_up)
                <= head_up(bx, by, bz, opt.tool_head_axis, world_up)
                else -plus
            )

    def make_pose(t, z_in):
        x, y, z = assemble(z_in)
        if abs(signed) > 1e-8:
            x, y, z = rotate_travel_angle(x, y, z, signed, vertical)
        T = np.eye(4, dtype=np.float64)
        T[:3, 0], T[:3, 1], T[:3, 2], T[:3, 3] = x, y, z, t
        return T

    return True, make_pose(t0, z0), make_pose(t1, z1)


class FlatVsVerticalTests(unittest.TestCase):
    def assert_orthonormal_rh(self, R: np.ndarray) -> None:
        np.testing.assert_allclose(R.T @ R, np.eye(3), atol=1e-9)
        self.assertGreater(float(np.linalg.det(R)), 0.0)
        np.testing.assert_allclose(R[:, 1], np.cross(R[:, 2], R[:, 0]), atol=1e-9)

    def test_rejects_fewer_than_two_points(self) -> None:
        self.assertFalse(compute_two_point_poses(PointCloudPointNormal())[0])

    def test_rejects_coincident_points(self) -> None:
        p = PointNormal(np.array([1.0, 2.0, 3.0]), WORLD_Z)
        self.assertFalse(compute_two_point_poses(PointCloudPointNormal([p, p]))[0])

    def test_auto_vertical_uses_y_bottom_to_top_and_horizontal_x(self) -> None:
        """立焊 PF：Y 从下到上沿缝，X 水平，鹅颈不朝天。"""
        n = np.array([0.0, 1.0, 0.0])
        low, high = np.zeros(3), np.array([0.0, 0.0, 2.0])
        traj = PointCloudPointNormal([PointNormal(high, n), PointNormal(low, n)])
        ok, Ts, Te = compute_two_point_poses(traj, Options(travel_angle_deg=0.0))
        self.assertTrue(ok)
        self.assertLess(Ts[2, 3], Te[2, 3])
        self.assertGreater(float(np.dot(Ts[:3, 1], WORLD_Z)), 0.5)
        self.assertGreater(float(np.dot(Te[:3, 1], WORLD_Z)), 0.5)
        self.assertLess(abs(float(np.dot(Ts[:3, 0], WORLD_Z))), 0.35)
        head = Ts[:3, :3] @ np.array([1.0, 0.0, 0.0])
        self.assertLess(abs(float(np.dot(head, WORLD_Z))), 0.35)
        self.assert_orthonormal_rh(Ts[:3, :3])
        self.assert_orthonormal_rh(Te[:3, :3])

    def test_flat_weld_keeps_x_as_travel(self) -> None:
        """平焊 PA：X 沿缝行走。"""
        t0 = np.zeros(3)
        weld = np.array([1.0, 0.2, 0.0])
        traj = PointCloudPointNormal(
            [PointNormal(t0, WORLD_Z), PointNormal(t0 + weld, WORLD_Z)]
        )
        opt = Options(travel_angle_deg=0.0, max_torch_tilt_deg=0.0)
        ok, Ts, _ = compute_two_point_poses(traj, opt)
        self.assertTrue(ok)
        hat = weld / np.linalg.norm(weld)
        self.assertGreater(abs(float(np.dot(Ts[:3, 0], hat))), 0.9)
        self.assert_orthonormal_rh(Ts[:3, :3])

    def test_force_flat_on_vertical_seam_still_uses_x_travel(self) -> None:
        n = np.array([0.0, 1.0, 0.0])
        t0, t1 = np.zeros(3), np.array([0.0, 0.0, 2.0])
        traj = PointCloudPointNormal([PointNormal(t0, n), PointNormal(t1, n)])
        opt = Options(
            weld_position=FLAT,
            travel_policy=KEEP,
            travel_angle_deg=0.0,
            max_torch_tilt_deg=0.0,
        )
        ok, Ts, _ = compute_two_point_poses(traj, opt)
        self.assertTrue(ok)
        self.assertGreater(float(np.dot(Ts[:3, 0], t1 - t0)), 0.5)

    def test_keep_given_vertical_does_not_swap_but_y_follows_travel(self) -> None:
        n = np.array([0.0, 1.0, 0.0])
        high, low = np.array([0.0, 0.0, 2.0]), np.zeros(3)
        traj = PointCloudPointNormal([PointNormal(high, n), PointNormal(low, n)])
        opt = Options(travel_policy=KEEP, travel_angle_deg=0.0, max_torch_tilt_deg=0.0)
        ok, Ts, Te = compute_two_point_poses(traj, opt)
        self.assertTrue(ok)
        self.assertGreater(Ts[2, 3], Te[2, 3])
        self.assertLess(float(np.dot(Ts[:3, 1], WORLD_Z)), 0.0)

    def test_inclined_floor_weld_projects_x_onto_torch_plane(self) -> None:
        z = WORLD_Z
        t0 = np.zeros(3)
        weld = np.array([1.0, 0.2, 0.5])
        traj = PointCloudPointNormal([PointNormal(t0, z), PointNormal(t0 + weld, z)])
        opt = Options(
            weld_position=FLAT,
            travel_policy=KEEP,
            travel_angle_deg=0.0,
            max_torch_tilt_deg=0.0,
        )
        ok, Ts, Te = compute_two_point_poses(traj, opt)
        self.assertTrue(ok)
        R = Ts[:3, :3]
        self.assert_orthonormal_rh(R)
        projected = weld - z * np.dot(weld, z)
        projected /= np.linalg.norm(projected)
        self.assertGreater(float(np.dot(R[:, 0], projected)), 0.9)
        self.assert_orthonormal_rh(Te[:3, :3])

    def test_vertical_travel_angle_keeps_x_horizontal(self) -> None:
        n = np.array([0.0, 1.0, 0.0])
        traj = PointCloudPointNormal(
            [PointNormal(np.zeros(3), n), PointNormal(np.array([0.0, 0.0, 2.0]), n)]
        )
        opt = Options(travel_angle_deg=15.0, max_torch_tilt_deg=0.0)
        ok, Ts, _ = compute_two_point_poses(traj, opt)
        self.assertTrue(ok)
        self.assertLess(abs(float(np.dot(Ts[:3, 0], WORLD_Z))), 0.2)
        self.assertGreater(float(np.dot(Ts[:3, 1], WORLD_Z)), 0.5)
        self.assert_orthonormal_rh(Ts[:3, :3])

    def test_flat_seam_optional_y_up_flip(self) -> None:
        z = np.array([0.0, 1.0, 0.0])
        t0 = np.zeros(3)
        weld = np.array([1.0, 0.0, 0.0])
        traj = PointCloudPointNormal([PointNormal(t0, z), PointNormal(t0 + weld, z)])
        opt = Options(
            weld_position=FLAT,
            travel_policy=KEEP,
            travel_angle_deg=0.0,
            max_torch_tilt_deg=0.0,
            flip_travel_to_raise_y=True,
        )
        ok, T, _ = compute_two_point_poses(traj, opt)
        self.assertTrue(ok)
        self.assertGreaterEqual(float(np.dot(T[:3, 1], WORLD_Z)), -1e-12)
        np.testing.assert_allclose(T[:3, 0], [-1.0, 0.0, 0.0], atol=1e-9)

    def test_table_normal_not_tilted_through_part_without_travel_angle(self) -> None:
        t0 = np.zeros(3)
        t1 = np.array([1.0, 0.0, 0.0])
        traj = PointCloudPointNormal([PointNormal(t0, WORLD_Z), PointNormal(t1, WORLD_Z)])
        opt = Options(travel_angle_deg=0.0)
        ok, Ts, _ = compute_two_point_poses(traj, opt)
        self.assertTrue(ok)
        np.testing.assert_allclose(Ts[:3, 2], WORLD_Z, atol=1e-9)

    def test_start_and_end_share_the_same_y_sign_on_vertical(self) -> None:
        n = np.array([0.0, 1.0, 0.0])
        traj = PointCloudPointNormal(
            [
                PointNormal(np.zeros(3), n),
                PointNormal(np.array([0.05, 0.0, 1.0]), n),
            ]
        )
        ok, Ts, Te = compute_two_point_poses(traj, Options(travel_angle_deg=0.0))
        self.assertTrue(ok)
        self.assertGreater(float(np.dot(Ts[:3, 1], Te[:3, 1])), 0.0)

    def test_travel_parallel_to_normal_still_orthonormal(self) -> None:
        weld = np.array([0.0, 0.0, 2.0])
        traj = PointCloudPointNormal(
            [PointNormal(np.zeros(3), WORLD_Z), PointNormal(weld, WORLD_Z)]
        )
        opt = Options(
            weld_position=FLAT,
            travel_policy=KEEP,
            travel_angle_deg=0.0,
            max_torch_tilt_deg=0.0,
        )
        ok, T, _ = compute_two_point_poses(traj, opt)
        self.assertTrue(ok)
        self.assert_orthonormal_rh(T[:3, :3])


if __name__ == "__main__":
    unittest.main()
