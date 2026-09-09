#!/usr/bin/env python3
"""Geometric contract for computeTwoPointPoses (mirrors include/ComputeTwoPointPoses.h).

PCL is not required: local normals use PCA with viewpoint at the origin, matching
pcl::NormalEstimation::setViewPoint(0,0,0). Pose axes follow the C++ makePose.
"""

from __future__ import annotations

import math
import unittest

import numpy as np

EPS = 1e-12


def make_pose(t: np.ndarray, z_in: np.ndarray, y_in: np.ndarray) -> np.ndarray:
    z = z_in / np.linalg.norm(z_in)
    x = np.cross(y_in, z)
    if np.dot(x, x) < EPS:
        fallback = np.array([0.0, 0.0, 1.0]) if abs(z[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
        x = np.cross(fallback, z)
    x = x / np.linalg.norm(x)
    y = np.cross(z, x)
    y = y / np.linalg.norm(y)
    T = np.eye(4, dtype=np.float64)
    T[:3, 0] = x
    T[:3, 1] = y
    T[:3, 2] = z
    T[:3, 3] = t
    return T


def pca_normal_toward_viewpoint(pts: np.ndarray, viewpoint: np.ndarray) -> np.ndarray | None:
    if pts.shape[0] < 3:
        return None
    center = pts.mean(axis=0)
    _, _, vh = np.linalg.svd(pts - center, full_matrices=False)
    n = vh[-1].copy()
    if np.dot(n, n) < EPS:
        return None
    n = n / np.linalg.norm(n)
    if np.dot(viewpoint - center, n) < 0.0:
        n = -n
    return n


def estimate_z(scene: np.ndarray, query: np.ndarray, radius: float) -> np.ndarray:
    """2R crop, R-neighborhood PCA normals, Z = -mean(inner-R normals)."""
    delta = scene - query
    d2 = np.einsum("ij,ij->i", delta, delta)
    r = float(radius)
    mask_2r = d2 <= (2.0 * r) ** 2
    local = scene[mask_2r]
    local_d2 = d2[mask_2r]
    if local.shape[0] == 0:
        return np.array([0.0, 0.0, 1.0])

    viewpoint = np.zeros(3, dtype=np.float64)
    normals = np.zeros_like(local)
    valid = np.zeros(local.shape[0], dtype=bool)
    r2 = r * r
    for i, p in enumerate(local):
        neigh = local[np.einsum("ij,ij->i", local - p, local - p) <= r2]
        n = pca_normal_toward_viewpoint(neigh, viewpoint)
        if n is None:
            continue
        normals[i] = n
        valid[i] = True

    def accumulate(inner_only: bool) -> np.ndarray:
        sel = valid.copy()
        if inner_only:
            sel &= local_d2 <= r2
        if not np.any(sel):
            return np.zeros(3)
        return normals[sel].sum(axis=0)

    s = accumulate(True)
    if not np.all(np.isfinite(s)) or np.dot(s, s) < EPS:
        s = accumulate(False)
    if not np.all(np.isfinite(s)) or np.dot(s, s) < EPS:
        return np.array([0.0, 0.0, 1.0])
    return -s / np.linalg.norm(s)


def compute_two_point_poses(
    start: np.ndarray, end: np.ndarray, scene: np.ndarray, radius: float
) -> tuple[np.ndarray, np.ndarray] | None:
    if radius <= 0.0 or scene.shape[0] == 0:
        return None
    y_dir = end - start
    if np.dot(y_dir, y_dir) < EPS:
        return None
    pose_start = make_pose(start, estimate_z(scene, start, radius), y_dir)
    pose_end = make_pose(end, estimate_z(scene, end, radius), y_dir)
    return pose_start, pose_end


class MakePoseTests(unittest.TestCase):
    def test_orthonormal_right_handed(self) -> None:
        T = make_pose(
            np.array([1.0, 2.0, 3.0]),
            np.array([0.0, 0.0, 1.0]),
            np.array([0.0, 1.0, 0.0]),
        )
        R = T[:3, :3]
        np.testing.assert_allclose(R.T @ R, np.eye(3), atol=1e-12)
        self.assertAlmostEqual(np.linalg.det(R), 1.0, places=12)
        np.testing.assert_allclose(T[:3, 3], [1.0, 2.0, 3.0])
        np.testing.assert_allclose(T[:3, 2], [0.0, 0.0, 1.0])
        np.testing.assert_allclose(T[:3, 1], [0.0, 1.0, 0.0])
        np.testing.assert_allclose(T[:3, 0], [1.0, 0.0, 0.0])

    def test_y_follows_start_end_when_orthogonal_to_z(self) -> None:
        t0 = np.array([0.0, 0.0, 1.0])
        t1 = np.array([0.2, 0.0, 1.0])
        z = np.array([0.0, 0.0, 1.0])
        T = make_pose(t0, z, t1 - t0)
        y = T[:3, 1]
        self.assertGreater(np.dot(y, t1 - t0), 0.0)
        self.assertLess(abs(np.dot(y, z)), 1e-12)

    def test_parallel_y_and_z_uses_fallback_and_stays_finite(self) -> None:
        T = make_pose(
            np.zeros(3),
            np.array([0.0, 0.0, 1.0]),
            np.array([0.0, 0.0, 2.0]),
        )
        R = T[:3, :3]
        self.assertTrue(np.all(np.isfinite(R)))
        np.testing.assert_allclose(R.T @ R, np.eye(3), atol=1e-12)
        self.assertAlmostEqual(np.linalg.det(R), 1.0, places=12)


class LocalNormalPoseTests(unittest.TestCase):
    def _plane_scene(self, z: float = 1.0, n: int = 21) -> np.ndarray:
        xs = np.linspace(-0.15, 0.15, n)
        ys = np.linspace(-0.15, 0.15, n)
        xx, yy = np.meshgrid(xs, ys)
        pts = np.stack([xx.ravel(), yy.ravel(), np.full(xx.size, z)], axis=1)
        return pts

    def test_z_points_into_plane_facing_camera(self) -> None:
        scene = self._plane_scene()
        start = np.array([-0.04, 0.0, 1.0])
        end = np.array([0.04, 0.0, 1.0])
        poses = compute_two_point_poses(start, end, scene, radius=0.03)
        self.assertIsNotNone(poses)
        pose_start, pose_end = poses
        # Viewpoint at origin orients PCA normals toward camera → (0,0,-1);
        # weld Z is the opposite, into the workpiece → (0,0,+1).
        np.testing.assert_allclose(pose_start[:3, 2], [0.0, 0.0, 1.0], atol=1e-6)
        np.testing.assert_allclose(pose_end[:3, 2], [0.0, 0.0, 1.0], atol=1e-6)
        np.testing.assert_allclose(pose_start[:3, 3], start)
        np.testing.assert_allclose(pose_end[:3, 3], end)
        self.assertGreater(np.dot(pose_start[:3, 1], end - start), 0.0)

    def test_rejects_coincident_start_end(self) -> None:
        scene = self._plane_scene()
        p = np.array([0.0, 0.0, 1.0])
        self.assertIsNone(compute_two_point_poses(p, p.copy(), scene, 0.03))

    def test_rejects_non_positive_radius(self) -> None:
        scene = self._plane_scene()
        self.assertIsNone(
            compute_two_point_poses(
                np.array([0.0, 0.0, 1.0]),
                np.array([0.05, 0.0, 1.0]),
                scene,
                0.0,
            )
        )


if __name__ == "__main__":
    unittest.main()
