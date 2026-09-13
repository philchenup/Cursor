"""Offline weld-seam pipeline tests on a synthetic V-groove."""

from __future__ import annotations

import unittest
from pathlib import Path
from tempfile import TemporaryDirectory

import numpy as np

from weld_seam_offline.pipeline import detect_weld_seam
from weld_seam_offline.ply_io import load_ply, save_ply
from weld_seam_offline.sample import make_vgroove_cloud, write_sample_ply
from weld_seam_offline.visualize import render_result
from weld_seam_offline.geometry import points_in_cylinder
from weld_seam_offline.pick import PickedPoints, picked_xyz, three_point_cylinder_roi


class PlyIoTests(unittest.TestCase):
    def test_roundtrip(self) -> None:
        pts = np.array([[0.0, 0.0, 0.0], [1.0, 2.0, 3.0], [-0.5, 0.25, 0.75]])
        with TemporaryDirectory() as tmp:
            path = Path(tmp) / "tiny.ply"
            save_ply(path, pts)
            loaded = load_ply(path)
        self.assertEqual(loaded.shape, (3, 3))
        np.testing.assert_allclose(loaded, pts, atol=1e-5)


class PipelineTests(unittest.TestCase):
    def test_detects_curved_vgroove(self) -> None:
        with TemporaryDirectory() as tmp:
            ply = write_sample_ply(Path(tmp) / "vgroove.ply")
            result = detect_weld_seam(ply)
            self.assertGreaterEqual(len(result.trajectory), 8)
            self.assertGreaterEqual(len(result.groove), 20)
            xs = result.trajectory[:, 0]
            self.assertGreater(xs.max() - xs.min(), 0.18)
            self.assertLess(abs(float(np.median(result.trajectory[:, 1]))), 0.03)
            self.assertEqual(result.poses.shape[1], 10)
            self.assertEqual(result.left_poses.shape[0], result.poses.shape[0])

            out = Path(tmp) / "out"
            paths = result.save(out)
            image = render_result(result, out / "trajectory.png")
            self.assertTrue(paths["trajectory_csv"].is_file())
            self.assertTrue(image.is_file())
            self.assertGreater(image.stat().st_size, 10_000)

    def test_array_input_matches_file(self) -> None:
        cloud = make_vgroove_cloud(rng=np.random.default_rng(1))
        with TemporaryDirectory() as tmp:
            ply = Path(tmp) / "arr.ply"
            save_ply(ply, cloud)
            from_file = detect_weld_seam(ply)
        from_array = detect_weld_seam(cloud)
        self.assertEqual(from_file.trajectory.shape[1], 3)
        self.assertEqual(from_array.trajectory.shape[1], 3)
        self.assertLessEqual(abs(len(from_file.trajectory) - len(from_array.trajectory)), 4)


class PickHelperTests(unittest.TestCase):
    def test_indices_to_xyz(self) -> None:
        class _Cloud:
            def __init__(self, points: np.ndarray) -> None:
                self.points = points

        pts = np.array([[0.0, 0.0, 0.0], [1.0, 2.0, 3.0], [4.0, 5.0, 6.0]])
        xyz = picked_xyz(_Cloud(pts), [2, 0])
        np.testing.assert_allclose(xyz, [[4.0, 5.0, 6.0], [0.0, 0.0, 0.0]])
        empty = picked_xyz(_Cloud(pts), [])
        self.assertEqual(empty.shape, (0, 3))

    def test_save_picked_csv(self) -> None:
        picked = PickedPoints(
            indices=np.array([3, 7]),
            xyz=np.array([[0.1, 0.2, 0.3], [0.4, 0.5, 0.6]]),
        )
        with TemporaryDirectory() as tmp:
            paths = picked.save(Path(tmp))
            loaded = np.loadtxt(paths["csv"], delimiter=",", skiprows=1)
        self.assertEqual(loaded.shape, (2, 4))
        np.testing.assert_allclose(loaded[0], [3, 0.1, 0.2, 0.3])

    def test_three_point_cylinder_roi(self) -> None:
        x = np.linspace(0.0, 1.0, 21)
        seam = np.column_stack([x, np.zeros(21), np.zeros(21)])
        noise = np.array([[0.5, 0.2, 0.0], [0.2, -0.25, 0.0]])
        cloud = np.vstack((seam, noise))
        roi = three_point_cylinder_roi(
            cloud,
            start=seam[0],
            corner=seam[10],
            end=seam[-1],
            radius=0.03,
        )
        self.assertGreaterEqual(len(roi), 15)
        self.assertTrue(np.all(np.abs(roi[:, 1]) < 0.05))
        inside = points_in_cylinder(seam[0], seam[-1], 0.02, cloud)
        self.assertGreaterEqual(len(inside), 15)


if __name__ == "__main__":
    unittest.main()
