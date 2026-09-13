#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Smoke tests for ROS-free welding_demo geometry helpers."""

import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from welding_demo import (  # noqa: E402
    _angle,
    mutilayer,
    points_in_cylinder,
    sort_points,
    thin_line,
    uplift_z,
)


class AngleTests(unittest.TestCase):
    def test_orthogonal(self):
        self.assertAlmostEqual(_angle([1, 0, 0], [0, 1, 0]), 90.0, places=6)

    def test_parallel(self):
        self.assertAlmostEqual(_angle([1, 0, 0], [2, 0, 0]), 0.0, places=6)


class ThinLineTests(unittest.TestCase):
    def test_projects_thick_line_onto_axis(self):
        rng = np.random.default_rng(0)
        t = np.linspace(0.0, 1.0, 40)
        center = np.column_stack((t, np.zeros_like(t), np.zeros_like(t)))
        noise = rng.normal(0.0, 0.01, size=center.shape)
        noise[:, 0] = 0.0
        points = center + noise
        thinned, lines = thin_line(points, point_cloud_thickness=0.05)
        self.assertEqual(thinned.shape, points.shape)
        self.assertEqual(len(lines), len(points))
        self.assertLess(np.std(thinned[:, 1]), np.std(points[:, 1]))


class SortPointsTests(unittest.TestCase):
    def test_orders_points_along_a_line(self):
        points = np.array([[i * 0.02, 0.0, 0.0] for i in range(12)], dtype=float)
        lines = []
        for p in points:
            lines.append([p - np.array([1.0, 0.0, 0.0]), p + np.array([1.0, 0.0, 0.0])])
        ordered = sort_points(points, lines, sorted_point_distance=0.02)
        self.assertGreaterEqual(len(ordered), 2)
        diffs = np.diff(ordered[:, 0])
        self.assertTrue(np.all(diffs > 0) or np.all(diffs < 0))


class CylinderTests(unittest.TestCase):
    def test_keeps_points_inside_cylinder(self):
        pt1 = np.array([0.0, 0.0, 0.0])
        pt2 = np.array([1.0, 0.0, 0.0])
        queries = np.array([
            [0.5, 0.0, 0.0],
            [0.5, 0.2, 0.0],
            [-0.1, 0.0, 0.0],
            [1.1, 0.0, 0.0],
        ])
        inside = points_in_cylinder(pt1, pt2, 0.05, queries)
        inside = np.asarray(inside)
        self.assertEqual(len(inside), 1)
        np.testing.assert_allclose(inside[0], [0.5, 0.0, 0.0])


class PoseOffsetTests(unittest.TestCase):
    def test_uplift_changes_translation_only(self):
        poses = [np.array([0.1, 0.2, 0.3, 0.0, 0.0, 0.0], dtype=float)]
        out = uplift_z(copy_poses(poses))
        self.assertEqual(len(out), 1)
        self.assertAlmostEqual(out[0][3], 0.0)
        self.assertNotAlmostEqual(out[0][2], 0.3)

    def test_mutilayer_returns_left_and_right(self):
        poses = [np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=float)]
        left, right = mutilayer(copy_poses(poses))
        self.assertEqual(len(left), 1)
        self.assertEqual(len(right), 1)
        self.assertFalse(np.allclose(left[0][:3], right[0][:3]))


def copy_poses(poses):
    return [p.copy() for p in poses]


class PipelineTests(unittest.TestCase):
    def test_detect_groove_on_synthetic_vgroove(self):
        import open3d as o3d
        import welding_demo as w

        rng = np.random.default_rng(1)
        pts = []
        for x in np.linspace(-0.15, 0.15, 90):
            for y in np.linspace(-0.12, 0.12, 70):
                if abs(y) < 0.018:
                    z = 0.42 - 0.012 * (1.0 - abs(y) / 0.018)
                else:
                    z = 0.42
                pts.append([x, y, z + rng.normal(0, 0.00025)])
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.asarray(pts))

        w.start = __import__("time").time()
        w.is_first_pose = True
        w.is_sec_pose = False
        w.capture_number = 0
        w.total_time = []
        w.max_dis = 0.7

        ur_poses = w.detect_groove_workflow(
            pcd,
            np.eye(4),
            show_groove=False,
            publish=False,
            save_data=False,
            apply_cam_to_base=False,
        )
        ur_poses = np.asarray(ur_poses)
        self.assertEqual(ur_poses.ndim, 2)
        self.assertEqual(ur_poses.shape[1], 6)
        self.assertGreaterEqual(ur_poses.shape[0], 2)
        # Trajectory should run mainly along X for this synthetic seam.
        self.assertGreater(np.ptp(ur_poses[:, 0]), np.ptp(ur_poses[:, 1]))


if __name__ == "__main__":
    unittest.main()
