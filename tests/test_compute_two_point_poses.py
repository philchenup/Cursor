#!/usr/bin/env python3
"""Ground workpiece: flat PA inward 30°, vertical PF bottom-to-top, Y-up order."""

from __future__ import annotations

import math
import unittest

import numpy as np

EPS = 1e-12
WORLD_Z = np.array([0.0, 0.0, 1.0])
PREFERRED_TORCH = np.array([0.0, 0.0, -1.0])
KEEP, DOWNHILL, UPHILL = "keep", "downhill", "uphill"
AUTO, FLAT, VERTICAL = "auto", "flat", "vertical"


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
        torch_x=None,
        x_align_hysteresis=0.2,
        inward_deg=30.0,
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
        self.torch_x = np.zeros(3) if torch_x is None else np.asarray(torch_x, dtype=np.float64)
        self.x_align_hysteresis = x_align_hysteresis
        self.inward_deg = inward_deg


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


def pose_at(xyz, z_axis) -> np.ndarray:
    T = np.eye(4, dtype=np.float64)
    T[:3, 3] = np.asarray(xyz, dtype=np.float64)
    T[:3, 2] = np.asarray(z_axis, dtype=np.float64)
    return T


def assemble_travel_frame(z_in, travel, world_up, vertical, flip_xy):
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


def swap_ends(t0, t1, z0, z1, travel):
    return t1.copy(), t0.copy(), z1.copy(), z0.copy(), -travel


def align_travel_so_y_up(travel, t0, t1, z0, z1, z_mean, world_up):
    y_probe = np.cross(z_mean, travel)
    if float(np.dot(y_probe, y_probe)) >= EPS and float(np.dot(y_probe, world_up)) < 0.0:
        t0, t1, z0, z1, travel = swap_ends(t0, t1, z0, z1, travel)
    return travel, t0, t1, z0, z1


def compute_two_point_poses(pose_start, pose_end, opt: Options | None = None):
    opt = opt or Options()
    t0 = pose_start[:3, 3].copy()
    t1 = pose_end[:3, 3].copy()
    weld = t1 - t0
    if float(np.dot(weld, weld)) < EPS:
        return False, None, None

    world_up = finite_unit(opt.world_up, WORLD_Z)
    seam = weld / np.linalg.norm(weld)
    vertical = is_vertical_seam(seam, world_up, opt)

    def prepare_z(z_in: np.ndarray) -> np.ndarray:
        n = finite_unit(np.asarray(z_in, dtype=np.float64), world_up)
        return tilt_torch_toward(n, opt.preferred_torch, opt.max_torch_tilt_deg)

    z0, z1 = prepare_z(pose_start[:3, 2]), prepare_z(pose_end[:3, 2])
    travel = seam.copy()
    if opt.travel_policy != KEEP:
        if vertical:
            du = float(np.dot(travel, world_up))
            if du < -1e-6:
                t0, t1, z0, z1, travel = swap_ends(t0, t1, z0, z1, travel)
        else:
            z_for_y = finite_unit(z0 + z1, z0)
            travel, t0, t1, z0, z1 = align_travel_so_y_up(
                seam, t0, t1, z0, z1, z_for_y, world_up
            )

    z_mean = finite_unit(z0 + z1, z0)
    if float(np.dot(opt.torch_x, opt.torch_x)) >= EPS:
        xref = project_perp(opt.torch_x, z_mean)
        if float(np.dot(xref, xref)) < EPS:
            xref = opt.torch_x
        xref = finite_unit(xref, travel)
        if float(np.dot(travel, xref)) < -opt.x_align_hysteresis:
            t0, t1, z0, z1, travel = swap_ends(t0, t1, z0, z1, travel)
            z_mean = finite_unit(z0 + z1, z0)
    if (not vertical) and opt.travel_policy != KEEP:
        travel, t0, t1, z0, z1 = align_travel_so_y_up(
            travel, t0, t1, z0, z1, z_mean, world_up
        )
        z_mean = finite_unit(z0 + z1, z0)

    steep = abs(float(np.dot(travel, world_up))) >= opt.steep_seam_abs_cos
    y_probe = np.cross(z_mean, travel)
    have_torch = float(np.dot(opt.torch_x, opt.torch_x)) >= EPS
    flip_xy = (
        (not vertical)
        and (not have_torch)
        and opt.flip_travel_to_raise_y
        and not steep
        and float(np.dot(y_probe, y_probe)) >= EPS
        and float(np.dot(y_probe, world_up)) < 0.0
    )

    def assemble(z_in):
        return assemble_travel_frame(z_in, travel, world_up, vertical, flip_xy)

    px, py, pz = assemble(z_mean)
    signed = 0.0
    if vertical:
        mag = abs(opt.travel_angle_deg) * math.pi / 180.0
        if mag > 1e-8:
            plus = mag if opt.travel_angle_deg >= 0.0 else -mag
            if not opt.gravity_signed_travel_angle:
                signed = plus
            else:
                ax, ay, az = rotate_travel_angle(px, py, pz, plus, True)
                bx, by, bz = rotate_travel_angle(px, py, pz, -plus, True)
                signed = (
                    plus
                    if head_up(ax, ay, az, opt.tool_head_axis, world_up)
                    <= head_up(bx, by, bz, opt.tool_head_axis, world_up)
                    else -plus
                )
    inward_rad = 0.0 if vertical else math.radians(opt.inward_deg)

    def make_pose(t, z_in, inward_sign):
        x, y, z = assemble(z_in)
        if not vertical and float(np.dot(opt.torch_x, opt.torch_x)) >= EPS:
            if float(np.dot(x, opt.torch_x)) < -opt.x_align_hysteresis:
                x, y, z = make_frame(-x, -y, z)
        if abs(inward_rad) > 1e-8:
            x, y, z = rotate_around_y(x, y, z, inward_sign * inward_rad)
        if abs(signed) > 1e-8:
            x, y, z = rotate_travel_angle(x, y, z, signed, vertical)
        T = np.eye(4, dtype=np.float64)
        T[:3, 0], T[:3, 1], T[:3, 2], T[:3, 3] = x, y, z, t
        return T

    return True, make_pose(t0, z0, -1.0), make_pose(t1, z1, 1.0)


def compute_weld_tcp_start_end(tcp_weld_start, tcp_weld_end, inward_deg=30.0):
    t0 = tcp_weld_start[:3, 3].copy()
    t1 = tcp_weld_end[:3, 3].copy()
    z0 = tcp_weld_start[:3, 2].copy()
    z1 = tcp_weld_end[:3, 2].copy()
    weld = t1 - t0
    if float(np.dot(weld, weld)) < EPS:
        return False, tcp_weld_start, tcp_weld_end

    opt = Options(inward_deg=inward_deg, travel_angle_deg=0.0, max_torch_tilt_deg=0.0)
    seam = weld / np.linalg.norm(weld)
    vertical = is_vertical_seam(seam, WORLD_Z, opt)
    travel = seam.copy()
    if vertical:
        if float(np.dot(travel, WORLD_Z)) < -1e-6:
            t0, t1, z0, z1, travel = swap_ends(t0, t1, z0, z1, travel)
    else:
        z_mean = finite_unit(z0 + z1, z0)
        travel, t0, t1, z0, z1 = align_travel_so_y_up(
            seam, t0, t1, z0, z1, z_mean, WORLD_Z
        )

    rad = 0.0 if vertical else math.radians(inward_deg)

    def pose_at_tilt(t, z_in, y_rad):
        x, y, z = assemble_travel_frame(z_in, travel, WORLD_Z, vertical, False)
        if (not vertical) and abs(y_rad) > 1e-8:
            x, y, z = rotate_around_y(x, y, z, y_rad)
        T = np.eye(4, dtype=np.float64)
        T[:3, 0], T[:3, 1], T[:3, 2], T[:3, 3] = x, y, z, t
        return T

    return True, pose_at_tilt(t0, z0, -rad), pose_at_tilt(t1, z1, rad)


class FlatVsVerticalTests(unittest.TestCase):
    def assert_orthonormal_rh(self, R: np.ndarray) -> None:
        np.testing.assert_allclose(R.T @ R, np.eye(3), atol=1e-9)
        self.assertGreater(float(np.linalg.det(R)), 0.0)
        np.testing.assert_allclose(R[:, 1], np.cross(R[:, 2], R[:, 0]), atol=1e-9)

    def untilted(self, **kwargs) -> Options:
        kw = dict(
            travel_angle_deg=0.0,
            max_torch_tilt_deg=0.0,
            inward_deg=0.0,
        )
        kw.update(kwargs)
        return Options(**kw)

    def test_rejects_coincident_points(self) -> None:
        T = pose_at([1.0, 2.0, 3.0], WORLD_Z)
        self.assertFalse(compute_two_point_poses(T, T)[0])

    def test_auto_vertical_uses_y_bottom_to_top_and_horizontal_x(self) -> None:
        """立焊 PF：Y 从下到上沿缝，X 水平，鹅颈不朝天。不内倾。"""
        n = np.array([0.0, 1.0, 0.0])
        low, high = np.zeros(3), np.array([0.0, 0.0, 2.0])
        ok, Ts, Te = compute_two_point_poses(
            pose_at(high, n), pose_at(low, n), self.untilted()
        )
        self.assertTrue(ok)
        self.assertLess(Ts[2, 3], Te[2, 3])
        self.assertGreater(float(np.dot(Ts[:3, 1], WORLD_Z)), 0.5)
        self.assertGreater(float(np.dot(Te[:3, 1], WORLD_Z)), 0.5)
        self.assertLess(abs(float(np.dot(Ts[:3, 0], WORLD_Z))), 0.35)
        head = Ts[:3, :3] @ np.array([1.0, 0.0, 0.0])
        self.assertLess(abs(float(np.dot(head, WORLD_Z))), 0.35)
        np.testing.assert_allclose(Ts[:3, 2], n, atol=1e-6)
        self.assert_orthonormal_rh(Ts[:3, :3])
        self.assert_orthonormal_rh(Te[:3, :3])

    def test_flat_weld_keeps_x_as_travel(self) -> None:
        """平焊 PA：X 沿缝行走。"""
        t0 = np.zeros(3)
        weld = np.array([1.0, 0.2, 0.0])
        ok, Ts, _ = compute_two_point_poses(
            pose_at(t0, WORLD_Z), pose_at(t0 + weld, WORLD_Z), self.untilted()
        )
        self.assertTrue(ok)
        hat = weld / np.linalg.norm(weld)
        self.assertGreater(abs(float(np.dot(Ts[:3, 0], hat))), 0.9)
        self.assert_orthonormal_rh(Ts[:3, :3])

    def test_flat_fillet_y_up_swaps_start_end(self) -> None:
        """角焊缝：Y = Z×X 朝上，反向输入会对调起终点。"""
        z = np.array([0.0, -math.sqrt(0.5), -math.sqrt(0.5)])
        a, b = np.zeros(3), np.array([1.0, 0.0, 0.0])
        opt = self.untilted(weld_position=FLAT)
        ok, Ts, Te = compute_two_point_poses(pose_at(b, z), pose_at(a, z), opt)
        self.assertTrue(ok)
        self.assertLess(Ts[0, 3], Te[0, 3])
        self.assertGreater(float(np.dot(Ts[:3, 1], WORLD_Z)), 0.5)
        self.assertGreater(float(np.dot(Te[:3, 1], WORLD_Z)), 0.5)
        self.assertGreater(float(np.dot(Te[:3, 3] - Ts[:3, 3], Ts[:3, 0])), 0.5)

    def test_force_flat_on_vertical_seam_still_uses_x_travel(self) -> None:
        n = np.array([0.0, 1.0, 0.0])
        t0, t1 = np.zeros(3), np.array([0.0, 0.0, 2.0])
        opt = self.untilted(weld_position=FLAT, travel_policy=KEEP)
        ok, Ts, _ = compute_two_point_poses(pose_at(t0, n), pose_at(t1, n), opt)
        self.assertTrue(ok)
        self.assertGreater(float(np.dot(Ts[:3, 0], t1 - t0)), 0.5)

    def test_keep_given_vertical_does_not_swap_but_y_follows_travel(self) -> None:
        n = np.array([0.0, 1.0, 0.0])
        high, low = np.array([0.0, 0.0, 2.0]), np.zeros(3)
        opt = self.untilted(travel_policy=KEEP)
        ok, Ts, Te = compute_two_point_poses(pose_at(high, n), pose_at(low, n), opt)
        self.assertTrue(ok)
        self.assertGreater(Ts[2, 3], Te[2, 3])
        self.assertLess(float(np.dot(Ts[:3, 1], WORLD_Z)), 0.0)

    def test_inclined_floor_weld_projects_x_onto_torch_plane(self) -> None:
        z = WORLD_Z
        t0 = np.zeros(3)
        weld = np.array([1.0, 0.2, 0.5])
        opt = self.untilted(weld_position=FLAT, travel_policy=KEEP)
        ok, Ts, Te = compute_two_point_poses(
            pose_at(t0, z), pose_at(t0 + weld, z), opt
        )
        self.assertTrue(ok)
        R = Ts[:3, :3]
        self.assert_orthonormal_rh(R)
        projected = weld - z * np.dot(weld, z)
        projected /= np.linalg.norm(projected)
        self.assertGreater(float(np.dot(R[:, 0], projected)), 0.9)
        self.assert_orthonormal_rh(Te[:3, :3])

    def test_vertical_travel_angle_keeps_x_horizontal(self) -> None:
        n = np.array([0.0, 1.0, 0.0])
        opt = Options(
            travel_angle_deg=15.0, max_torch_tilt_deg=0.0, inward_deg=30.0
        )
        ok, Ts, _ = compute_two_point_poses(
            pose_at(np.zeros(3), n), pose_at(np.array([0.0, 0.0, 2.0]), n), opt
        )
        self.assertTrue(ok)
        self.assertLess(abs(float(np.dot(Ts[:3, 0], WORLD_Z))), 0.2)
        self.assertGreater(float(np.dot(Ts[:3, 1], WORLD_Z)), 0.5)
        self.assert_orthonormal_rh(Ts[:3, :3])

    def test_flat_seam_optional_y_up_flip(self) -> None:
        z = np.array([0.0, 1.0, 0.0])
        t0 = np.zeros(3)
        weld = np.array([1.0, 0.0, 0.0])
        opt = self.untilted(
            weld_position=FLAT,
            travel_policy=KEEP,
            flip_travel_to_raise_y=True,
        )
        ok, T, _ = compute_two_point_poses(
            pose_at(t0, z), pose_at(t0 + weld, z), opt
        )
        self.assertTrue(ok)
        self.assertGreaterEqual(float(np.dot(T[:3, 1], WORLD_Z)), -1e-12)
        np.testing.assert_allclose(T[:3, 0], [-1.0, 0.0, 0.0], atol=1e-9)

    def test_table_normal_not_tilted_through_part_without_travel_angle(self) -> None:
        opt = self.untilted()
        ok, Ts, _ = compute_two_point_poses(
            pose_at(np.zeros(3), WORLD_Z),
            pose_at(np.array([1.0, 0.0, 0.0]), WORLD_Z),
            opt,
        )
        self.assertTrue(ok)
        np.testing.assert_allclose(Ts[:3, 2], WORLD_Z, atol=1e-9)

    def test_start_and_end_share_the_same_y_sign_on_vertical(self) -> None:
        n = np.array([0.0, 1.0, 0.0])
        ok, Ts, Te = compute_two_point_poses(
            pose_at(np.zeros(3), n),
            pose_at(np.array([0.05, 0.0, 1.0]), n),
            self.untilted(),
        )
        self.assertTrue(ok)
        self.assertGreater(float(np.dot(Ts[:3, 1], Te[:3, 1])), 0.0)

    def test_travel_parallel_to_normal_still_orthonormal(self) -> None:
        weld = np.array([0.0, 0.0, 2.0])
        opt = self.untilted(weld_position=FLAT, travel_policy=KEEP)
        ok, T, _ = compute_two_point_poses(
            pose_at(np.zeros(3), WORLD_Z), pose_at(weld, WORLD_Z), opt
        )
        self.assertTrue(ok)
        self.assert_orthonormal_rh(T[:3, :3])

    def test_torch_x_keeps_positive_when_start_end_reversed(self) -> None:
        """起终点反了也不翻枪 X：路径跟着焊枪 +X。"""
        n = WORLD_Z
        a, b = np.zeros(3), np.array([1.0, 0.0, 0.0])
        torch_x = np.array([1.0, 0.0, 0.0])
        opt = self.untilted(
            weld_position=FLAT,
            travel_policy=KEEP,
            torch_x=torch_x,
        )
        _, Tf, _ = compute_two_point_poses(pose_at(a, n), pose_at(b, n), opt)
        _, Tr, _ = compute_two_point_poses(pose_at(b, n), pose_at(a, n), opt)
        self.assertGreater(float(np.dot(Tf[:3, 0], torch_x)), 0.5)
        self.assertGreater(float(np.dot(Tr[:3, 0], torch_x)), 0.5)
        self.assertGreater(float(np.dot(Tf[:3, 0], Tr[:3, 0])), 0.5)

    def test_torch_x_hysteresis_does_not_flip_near_perpendicular(self) -> None:
        n = WORLD_Z
        a, b = np.zeros(3), np.array([1.0, 0.0, 0.0])
        opt = self.untilted(
            weld_position=FLAT,
            travel_policy=KEEP,
            torch_x=np.array([0.0, 1.0, 0.0]),
            x_align_hysteresis=0.2,
        )
        _, T, _ = compute_two_point_poses(pose_at(a, n), pose_at(b, n), opt)
        self.assertGreater(float(np.dot(T[:3, 0], b - a)), 0.5)

    def test_flat_default_inward_30_does_not_lean_into_side_walls(self) -> None:
        z = np.array([0.0, -math.sqrt(0.5), -math.sqrt(0.5)])
        opt = Options(
            weld_position=FLAT,
            max_torch_tilt_deg=0.0,
            travel_angle_deg=0.0,
            inward_deg=30.0,
        )
        ok, Ts, Te = compute_two_point_poses(
            pose_at(np.zeros(3), z), pose_at(np.array([1.0, 0.0, 0.0]), z), opt
        )
        self.assertTrue(ok)
        travel = Te[:3, 3] - Ts[:3, 3]
        travel = travel / np.linalg.norm(travel)
        self.assertGreater(float(np.dot(Ts[:3, 2], travel)), 0.3)
        self.assertLess(float(np.dot(Te[:3, 2], travel)), -0.3)
        self.assertGreater(float(np.dot(Ts[:3, 1], WORLD_Z)), 0.4)
        np.testing.assert_allclose(Ts[:3, 1], Te[:3, 1], atol=1e-6)
        self.assertLess(abs(float(np.dot(Ts[:3, 2], Ts[:3, 1]))), 1e-6)
        self.assertLess(abs(float(np.dot(Te[:3, 2], Te[:3, 1]))), 1e-6)
        self.assert_orthonormal_rh(Ts[:3, :3])
        self.assert_orthonormal_rh(Te[:3, :3])


class WeldTcpStartEndTests(unittest.TestCase):
    def assert_orthonormal_rh(self, R: np.ndarray) -> None:
        np.testing.assert_allclose(R.T @ R, np.eye(3), atol=1e-9)
        self.assertGreater(float(np.linalg.det(R)), 0.0)

    def test_flat_y_up_orders_start_end_not_near_to_far(self) -> None:
        z = np.array([0.0, -math.sqrt(0.5), -math.sqrt(0.5)])
        far = pose_at([200.0, 0.0, 0.0], z)
        near = pose_at([50.0, 0.0, 0.0], z)
        ok, Ts, Te = compute_weld_tcp_start_end(far, near)
        self.assertTrue(ok)
        self.assertLess(Ts[0, 3], Te[0, 3])
        self.assertGreater(float(np.dot(Ts[:3, 1], WORLD_Z)), 0.4)
        self.assertGreater(float(np.dot(Te[:3, 1], WORLD_Z)), 0.4)
        travel = Te[:3, 3] - Ts[:3, 3]
        self.assertGreater(float(np.dot(Ts[:3, 0], travel)), 0.0)
        self.assertGreater(float(np.dot(Te[:3, 0], travel)), 0.0)

    def test_flat_both_ends_tilt_inward_30_around_y(self) -> None:
        z = np.array([0.0, -math.sqrt(0.5), -math.sqrt(0.5)])
        ok, Ts, Te = compute_weld_tcp_start_end(
            pose_at([0.0, 0.0, 0.0], z),
            pose_at([100.0, 0.0, 0.0], z),
        )
        self.assertTrue(ok)
        travel = Te[:3, 3] - Ts[:3, 3]
        travel /= np.linalg.norm(travel)
        self.assertGreater(float(np.dot(Ts[:3, 2], travel)), 0.3)
        self.assertLess(float(np.dot(Te[:3, 2], travel)), -0.3)
        self.assertAlmostEqual(
            math.degrees(math.acos(float(np.clip(np.dot(Ts[:3, 2], z), -1.0, 1.0)))),
            30.0,
            places=4,
        )
        self.assertAlmostEqual(
            math.degrees(math.acos(float(np.clip(np.dot(Te[:3, 2], z), -1.0, 1.0)))),
            30.0,
            places=4,
        )
        self.assert_orthonormal_rh(Ts[:3, :3])
        self.assert_orthonormal_rh(Te[:3, :3])

    def test_flat_does_not_lean_into_side_walls(self) -> None:
        z = WORLD_Z
        ok, Ts, Te = compute_weld_tcp_start_end(
            pose_at([0.0, 0.0, 0.0], z),
            pose_at([100.0, 0.0, 0.0], z),
        )
        self.assertTrue(ok)
        side = np.array([0.0, 1.0, 0.0])
        self.assertGreater(abs(float(np.dot(Ts[:3, 1], side))), 0.99)
        self.assertGreater(abs(float(np.dot(Te[:3, 1], side))), 0.99)
        self.assertLess(abs(float(np.dot(Ts[:3, 2], side))), 1e-6)
        self.assertLess(abs(float(np.dot(Te[:3, 2], side))), 1e-6)

    def test_vertical_bottom_to_top_without_inward_tilt(self) -> None:
        n = np.array([0.0, 1.0, 0.0])
        high = pose_at([0.0, 0.0, 2.0], n)
        low = pose_at([0.0, 0.0, 0.0], n)
        ok, Ts, Te = compute_weld_tcp_start_end(high, low)
        self.assertTrue(ok)
        self.assertLess(Ts[2, 3], Te[2, 3])
        self.assertGreater(float(np.dot(Ts[:3, 1], WORLD_Z)), 0.5)
        self.assertGreater(float(np.dot(Te[:3, 1], WORLD_Z)), 0.5)
        self.assertLess(abs(float(np.dot(Ts[:3, 0], WORLD_Z))), 0.2)
        np.testing.assert_allclose(Ts[:3, 2], n, atol=1e-6)
        np.testing.assert_allclose(Te[:3, 2], n, atol=1e-6)
        self.assert_orthonormal_rh(Ts[:3, :3])
        self.assert_orthonormal_rh(Te[:3, :3])


if __name__ == "__main__":
    unittest.main()
