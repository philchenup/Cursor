#!/usr/bin/env python3
"""Teach-less weld poses: downhill travel + gravity-signed torch head."""

from __future__ import annotations

import math
import unittest

import numpy as np

EPS = 1e-12
WORLD_Z = np.array([0.0, 0.0, 1.0])
PREFERRED_TORCH = np.array([0.0, 0.0, -1.0])
KEEP, DOWNHILL, UPHILL = "keep", "downhill", "uphill"


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


def make_frame(x: np.ndarray, y: np.ndarray, z: np.ndarray):
    x = finite_unit(x, np.array([1.0, 0.0, 0.0]))
    y = finite_unit(y, np.array([0.0, 1.0, 0.0]))
    z = finite_unit(np.cross(x, y), z)
    y = finite_unit(np.cross(z, x), y)
    return x, y, z


def rotate_around_y(x, y, z, rad):
    c, s = math.cos(rad), math.sin(rad)
    return make_frame(x * c + z * s, y, -x * s + z * c)


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

    def prepare_z(q: PointNormal) -> np.ndarray:
        n = np.array([q.normal_x, q.normal_y, q.normal_z], dtype=np.float64)
        n = finite_unit(n, world_up)
        return tilt_torch_toward(n, opt.preferred_torch, opt.max_torch_tilt_deg)

    z0, z1 = prepare_z(p0), prepare_z(p1)
    travel = weld / np.linalg.norm(weld)
    du = float(np.dot(travel, world_up))
    swap = (opt.travel_policy == DOWNHILL and du > 1e-6) or (
        opt.travel_policy == UPHILL and du < -1e-6
    )
    if swap:
        t0, t1 = t1.copy(), t0.copy()
        z0, z1 = z1.copy(), z0.copy()
        travel = -travel

    steep = abs(float(np.dot(travel, world_up))) >= opt.steep_seam_abs_cos
    z_mean = finite_unit(z0 + z1, z0)
    y_probe = np.cross(z_mean, travel)
    flip_xy = (
        opt.flip_travel_to_raise_y
        and not steep
        and float(np.dot(y_probe, y_probe)) >= EPS
        and float(np.dot(y_probe, world_up)) < 0.0
    )

    def assemble(z_in):
        z = finite_unit(z_in, world_up)
        x = travel_axis_in_torch_plane(z, travel, world_up)
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
            ax, ay, az = rotate_around_y(px, py, pz, plus)
            bx, by, bz = rotate_around_y(px, py, pz, -plus)
            signed = (
                plus
                if head_up(ax, ay, az, opt.tool_head_axis, world_up)
                <= head_up(bx, by, bz, opt.tool_head_axis, world_up)
                else -plus
            )

    def make_pose(t, z_in):
        x, y, z = assemble(z_in)
        if abs(signed) > 1e-8:
            x, y, z = rotate_around_y(x, y, z, signed)
        T = np.eye(4, dtype=np.float64)
        T[:3, 0], T[:3, 1], T[:3, 2], T[:3, 3] = x, y, z, t
        return T

    return True, make_pose(t0, z0), make_pose(t1, z1)


class TeachlessTorchTests(unittest.TestCase):
    def assert_orthonormal_rh(self, R: np.ndarray) -> None:
        np.testing.assert_allclose(R.T @ R, np.eye(3), atol=1e-9)
        self.assertGreater(float(np.linalg.det(R)), 0.0)
        np.testing.assert_allclose(R[:, 1], np.cross(R[:, 2], R[:, 0]), atol=1e-9)

    def test_rejects_fewer_than_two_points(self) -> None:
        self.assertFalse(compute_two_point_poses(PointCloudPointNormal())[0])

    def test_rejects_coincident_points(self) -> None:
        p = PointNormal(np.array([1.0, 2.0, 3.0]), WORLD_Z)
        self.assertFalse(compute_two_point_poses(PointCloudPointNormal([p, p]))[0])

    def test_downhill_policy_puts_torch_head_down_on_vertical_wall(self) -> None:
        """免示教：立缝输入从低到高时，改为高→低，枪头（+X）朝下。"""
        n = np.array([0.0, 1.0, 0.0])
        low = np.array([0.0, 0.0, 0.0])
        high = np.array([0.0, 0.0, 2.0])
        traj = PointCloudPointNormal([PointNormal(low, n), PointNormal(high, n)])
        ok, Ts, Te = compute_two_point_poses(traj)
        self.assertTrue(ok)
        self.assertGreater(Ts[2, 3], Te[2, 3])
        self.assertLess(float(np.dot(Ts[:3, 0], WORLD_Z)), 0.0)
        self.assertLess(float(np.dot(Te[:3, 0], WORLD_Z)), 0.0)
        self.assert_orthonormal_rh(Ts[:3, :3])
        self.assert_orthonormal_rh(Te[:3, :3])

    def test_keep_given_preserves_upward_travel(self) -> None:
        n = np.array([0.0, 1.0, 0.0])
        t0 = np.zeros(3)
        t1 = np.array([0.0, 0.0, 2.0])
        traj = PointCloudPointNormal([PointNormal(t0, n), PointNormal(t1, n)])
        opt = Options(travel_policy=KEEP, travel_angle_deg=0.0)
        ok, Ts, Te = compute_two_point_poses(traj, opt)
        self.assertTrue(ok)
        self.assertGreater(float(np.dot(Ts[:3, 0], t1 - t0)), 0.0)
        self.assertGreater(float(np.dot(Te[:3, 0], t1 - t0)), 0.0)

    def test_uphill_policy_starts_at_lower_point(self) -> None:
        n = np.array([0.0, 1.0, 0.0])
        high = np.array([0.0, 0.0, 2.0])
        low = np.array([0.0, 0.0, 0.0])
        traj = PointCloudPointNormal([PointNormal(high, n), PointNormal(low, n)])
        opt = Options(travel_policy=UPHILL, travel_angle_deg=0.0, max_torch_tilt_deg=0.0)
        ok, Ts, Te = compute_two_point_poses(traj, opt)
        self.assertTrue(ok)
        self.assertLess(Ts[2, 3], Te[2, 3])
        self.assertGreater(float(np.dot(Ts[:3, 0], WORLD_Z)), 0.0)

    def test_gravity_travel_angle_picks_the_more_downward_sign(self) -> None:
        """行走角符号跟重力：在 +α/−α 里取枪头更朝下的一侧，不跟 X 正负绑定。"""
        n = np.array([0.0, 1.0, 0.0])
        low, high = np.zeros(3), np.array([0.0, 0.0, 2.0])
        head = np.array([1.0, 0.0, 0.0])
        for pts in ((low, high), (high, low)):
            traj = PointCloudPointNormal([PointNormal(pts[0], n), PointNormal(pts[1], n)])
            base = dict(
                travel_policy=KEEP,
                max_torch_tilt_deg=0.0,
                tool_head_axis=head,
            )
            _, P0, _ = compute_two_point_poses(
                traj, Options(**base, travel_angle_deg=0.0)
            )
            plus = Options(
                **base, travel_angle_deg=15.0, gravity_signed_travel_angle=False
            )
            minus = Options(
                **base, travel_angle_deg=-15.0, gravity_signed_travel_angle=False
            )
            auto = Options(
                **base, travel_angle_deg=15.0, gravity_signed_travel_angle=True
            )
            _, Pp, _ = compute_two_point_poses(traj, plus)
            _, Pm, _ = compute_two_point_poses(traj, minus)
            _, Pa, _ = compute_two_point_poses(traj, auto)
            up_p = float(np.dot(Pp[:3, :3] @ head, WORLD_Z))
            up_m = float(np.dot(Pm[:3, :3] @ head, WORLD_Z))
            up_a = float(np.dot(Pa[:3, :3] @ head, WORLD_Z))
            self.assertAlmostEqual(up_a, min(up_p, up_m), places=6)
            if float(np.dot(P0[:3, 0], WORLD_Z)) > 0.0:
                self.assertLess(up_a, float(np.dot(P0[:3, :3] @ head, WORLD_Z)))
            self.assert_orthonormal_rh(Pa[:3, :3])

    def test_inclined_floor_weld_projects_x_onto_torch_plane(self) -> None:
        z = WORLD_Z
        t0 = np.zeros(3)
        weld = np.array([1.0, 0.2, 0.5])
        traj = PointCloudPointNormal([PointNormal(t0, z), PointNormal(t0 + weld, z)])
        opt = Options(travel_policy=KEEP, travel_angle_deg=0.0, max_torch_tilt_deg=0.0)
        ok, Ts, Te = compute_two_point_poses(traj, opt)
        self.assertTrue(ok)
        R = Ts[:3, :3]
        self.assert_orthonormal_rh(R)
        self.assertAlmostEqual(float(np.dot(R[:, 0], R[:, 2])), 0.0, places=6)
        projected = weld - z * np.dot(weld, z)
        projected /= np.linalg.norm(projected)
        self.assertGreater(float(np.dot(R[:, 0], projected)), 0.9)
        self.assert_orthonormal_rh(Te[:3, :3])

    def test_steep_seam_does_not_reverse_travel_to_raise_y(self) -> None:
        n = np.array([1.0, 0.0, 0.0])
        t0 = np.zeros(3)
        t1 = np.array([0.1, 0.0, 1.0])
        traj = PointCloudPointNormal([PointNormal(t0, n), PointNormal(t1, n)])
        opt = Options(
            travel_policy=KEEP,
            travel_angle_deg=0.0,
            max_torch_tilt_deg=0.0,
            flip_travel_to_raise_y=True,
        )
        ok, Ts, _ = compute_two_point_poses(traj, opt)
        self.assertTrue(ok)
        self.assertGreater(float(np.dot(Ts[:3, 0], t1 - t0)), 0.0)

    def test_start_and_end_share_the_same_x_sign(self) -> None:
        t0 = np.array([0.0, 0.0, 1.0])
        t1 = np.array([0.4, 0.1, 0.9])
        traj = PointCloudPointNormal(
            [
                PointNormal(t0, np.array([0.0, 1.0, 0.2])),
                PointNormal(t1, np.array([0.1, 0.9, -0.2])),
            ]
        )
        ok, Ts, Te = compute_two_point_poses(traj)
        self.assertTrue(ok)
        self.assertGreater(float(np.dot(Ts[:3, 0], Te[:3, 0])), 0.0)
        self.assert_orthonormal_rh(Ts[:3, :3])
        self.assert_orthonormal_rh(Te[:3, :3])

    def test_table_normal_not_tilted_through_part_without_travel_angle(self) -> None:
        t0 = np.zeros(3)
        t1 = np.array([1.0, 0.0, 0.0])
        traj = PointCloudPointNormal([PointNormal(t0, WORLD_Z), PointNormal(t1, WORLD_Z)])
        opt = Options(travel_angle_deg=0.0)
        ok, Ts, _ = compute_two_point_poses(traj, opt)
        self.assertTrue(ok)
        np.testing.assert_allclose(Ts[:3, 2], WORLD_Z, atol=1e-9)

    def test_flat_seam_optional_y_up_flip(self) -> None:
        z = np.array([0.0, 1.0, 0.0])
        t0 = np.zeros(3)
        weld = np.array([1.0, 0.0, 0.0])
        traj = PointCloudPointNormal([PointNormal(t0, z), PointNormal(t0 + weld, z)])
        opt = Options(
            travel_policy=KEEP,
            travel_angle_deg=0.0,
            max_torch_tilt_deg=0.0,
            flip_travel_to_raise_y=True,
        )
        ok, T, _ = compute_two_point_poses(traj, opt)
        self.assertTrue(ok)
        self.assertGreaterEqual(float(np.dot(T[:3, 1], WORLD_Z)), -1e-12)
        np.testing.assert_allclose(T[:3, 0], [-1.0, 0.0, 0.0], atol=1e-9)

    def test_travel_parallel_to_normal_still_orthonormal(self) -> None:
        weld = np.array([0.0, 0.0, 2.0])
        traj = PointCloudPointNormal(
            [PointNormal(np.zeros(3), WORLD_Z), PointNormal(weld, WORLD_Z)]
        )
        opt = Options(travel_policy=KEEP, travel_angle_deg=0.0, max_torch_tilt_deg=0.0)
        ok, T, _ = compute_two_point_poses(traj, opt)
        self.assertTrue(ok)
        self.assert_orthonormal_rh(T[:3, :3])
        self.assertAlmostEqual(float(np.dot(T[:3, 0], WORLD_Z)), 0.0, places=6)


if __name__ == "__main__":
    unittest.main()
