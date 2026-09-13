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


if __name__ == "__main__":
    unittest.main()
