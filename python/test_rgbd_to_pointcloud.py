#!/usr/bin/env python3
"""RGB-D 点云恢复的合成数据测试。"""

from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parent))

from rgbd_to_pointcloud import (
    CameraIntrinsics,
    generate_demo_rgbd,
    load_depth_png16,
    load_rgb_png,
    reconstruct_point_cloud,
    save_demo_images,
    save_point_cloud,
)


class TestRgbdToPointCloud(unittest.TestCase):
    def setUp(self) -> None:
        self.rgb, self.depth, self.intrinsics = generate_demo_rgbd(
            width=80, height=60, fx=70.0, fy=70.0, plane_z_m=1.5, depth_scale=1000.0
        )

    def test_load_16bit_png_roundtrip(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            rgb_path, depth_path = save_demo_images(self.rgb, self.depth, tmp)
            rgb = load_rgb_png(rgb_path)
            depth = load_depth_png16(depth_path)
            np.testing.assert_array_equal(rgb, self.rgb)
            np.testing.assert_array_equal(depth, self.depth)
            self.assertEqual(depth.dtype, np.uint16)

    def test_open3d_plane_depth(self) -> None:
        pcd = reconstruct_point_cloud(
            self.rgb, self.depth, self.intrinsics, depth_scale=1000.0, backend="open3d"
        )
        pts = np.asarray(pcd.points)
        self.assertGreater(len(pts), 1000)
        # 平面深度 1.5 m，四周 4 像素被置 0
        np.testing.assert_allclose(pts[:, 2], 1.5, atol=1e-6)
        colors = np.asarray(pcd.colors)
        self.assertEqual(len(colors), len(pts))
        self.assertTrue(np.all((colors >= 0.0) & (colors <= 1.0)))

    def test_numpy_backend_matches_open3d(self) -> None:
        a = reconstruct_point_cloud(
            self.rgb, self.depth, self.intrinsics, backend="open3d"
        )
        b = reconstruct_point_cloud(
            self.rgb, self.depth, self.intrinsics, backend="numpy"
        )
        pa = np.asarray(a.points)
        pb = np.asarray(b.points)
        self.assertEqual(len(pa), len(pb))
        # 两边都是按行优先扫像素，应对齐
        np.testing.assert_allclose(pa, pb, atol=1e-6)
        np.testing.assert_allclose(np.asarray(a.colors), np.asarray(b.colors), atol=1e-6)

    def test_write_ply_and_reload(self) -> None:
        pcd = reconstruct_point_cloud(self.rgb, self.depth, self.intrinsics)
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / "cloud.ply"
            save_point_cloud(pcd, out)
            self.assertTrue(out.is_file())
            self.assertGreater(out.stat().st_size, 0)
            import open3d as o3d

            loaded = o3d.io.read_point_cloud(str(out))
            self.assertEqual(len(loaded.points), len(pcd.points))

    def test_intrinsics_from_k_matrix(self) -> None:
        data = {
            "K": [[70.0, 0.0, 39.5], [0.0, 70.0, 29.5], [0.0, 0.0, 1.0]],
            "width": 80,
            "height": 60,
        }
        k = CameraIntrinsics.from_dict(data)
        self.assertAlmostEqual(k.fx, 70.0)
        self.assertAlmostEqual(k.cx, 39.5)
        self.assertAlmostEqual(k.cy, 29.5)

    def test_zero_depth_is_dropped(self) -> None:
        depth = np.zeros_like(self.depth)
        pcd = reconstruct_point_cloud(self.rgb, depth, self.intrinsics)
        self.assertEqual(len(pcd.points), 0)

    def test_cli_demo(self) -> None:
        from rgbd_to_pointcloud import main

        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / "demo.ply"
            code = main(
                [
                    "--demo",
                    "--demo-dir",
                    tmp,
                    "--output",
                    str(out),
                    "--backend",
                    "open3d",
                ]
            )
            self.assertEqual(code, 0)
            self.assertTrue(out.is_file())
            self.assertTrue((Path(tmp) / "demo_rgb.png").is_file())
            self.assertTrue((Path(tmp) / "demo_depth16.png").is_file())
            depth = np.array(Image.open(Path(tmp) / "demo_depth16.png"))
            self.assertEqual(depth.dtype, np.uint16)
            meta = json.loads((Path(tmp) / "demo_intrinsics.json").read_text(encoding="utf-8"))
            self.assertIn("fx", meta)


if __name__ == "__main__":
    unittest.main()
