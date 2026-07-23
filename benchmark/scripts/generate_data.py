#!/usr/bin/env python3
"""Generate shared synthetic point-cloud datasets for fair cross-library timing."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import open3d as o3d

ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / "data"
N_POINTS = 100_000
SEED = 42


def make_bunny_like_cloud(n: int, seed: int) -> o3d.geometry.PointCloud:
    rng = np.random.default_rng(seed)
    # Mixture of a plane + sphere + box corners for geometric richness
    plane = np.column_stack(
        [
            rng.uniform(-1.0, 1.0, n // 3),
            rng.uniform(-1.0, 1.0, n // 3),
            rng.normal(0.0, 0.01, n // 3),
        ]
    )
    phi = rng.uniform(0, 2 * np.pi, n // 3)
    costheta = rng.uniform(-1, 1, n // 3)
    u = rng.uniform(0, 1, n // 3)
    theta = np.arccos(costheta)
    r = 0.45 * u ** (1 / 3)
    sphere = np.column_stack(
        [
            r * np.sin(theta) * np.cos(phi) + 0.2,
            r * np.sin(theta) * np.sin(phi) - 0.1,
            r * np.cos(theta) + 0.35,
        ]
    )
    box = rng.uniform(-0.6, 0.6, (n - 2 * (n // 3), 3))
    box[:, 2] = np.abs(box[:, 2]) * 0.3 + 0.05
    pts = np.vstack([plane, sphere, box]).astype(np.float64)
    colors = rng.uniform(0.1, 0.95, pts.shape).astype(np.float64)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)
    pcd.colors = o3d.utility.Vector3dVector(colors)
    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.05, max_nn=30)
    )
    return pcd


def make_depth_and_rgbd(w: int = 320, h: int = 240, seed: int = 42):
    rng = np.random.default_rng(seed)
    yy, xx = np.mgrid[0:h, 0:w]
    cx, cy = w / 2.0, h / 2.0
    depth = (
        1.2
        + 0.15 * np.sin(xx / 18.0)
        + 0.12 * np.cos(yy / 22.0)
        + rng.normal(0, 0.002, (h, w))
    ).astype(np.float32)
    # Invalid pixels
    depth[(xx - cx) ** 2 + (yy - cy) ** 2 > (min(w, h) * 0.48) ** 2] = 0
    depth_u16 = np.clip(depth * 1000.0, 0, 65535).astype(np.uint16)
    color = np.zeros((h, w, 3), dtype=np.uint8)
    color[..., 0] = np.clip(120 + 80 * np.sin(xx / 25.0), 0, 255).astype(np.uint8)
    color[..., 1] = np.clip(100 + 60 * np.cos(yy / 30.0), 0, 255).astype(np.uint8)
    color[..., 2] = np.clip(90 + 70 * ((xx + yy) % 50) / 50.0, 0, 255).astype(np.uint8)
    return depth_u16, color, depth


def main() -> None:
    DATA.mkdir(parents=True, exist_ok=True)
    pcd = make_bunny_like_cloud(N_POINTS, SEED)
    pts = np.asarray(pcd.points)

    ply_path = DATA / "cloud.ply"
    pcd_path = DATA / "cloud.pcd"
    txt_path = DATA / "cloud.txt"

    def write_xyz_only(path_ply: Path, path_pcd: Path, cloud: o3d.geometry.PointCloud) -> None:
        xyz = o3d.geometry.PointCloud()
        xyz.points = cloud.points
        o3d.io.write_point_cloud(str(path_ply), xyz, write_ascii=False)
        arr = np.asarray(cloud.points, dtype=np.float32)
        with path_pcd.open("w", encoding="utf-8") as f:
            f.write("# .PCD v0.7 - Point Cloud Data file format\n")
            f.write("VERSION 0.7\n")
            f.write("FIELDS x y z\n")
            f.write("SIZE 4 4 4\n")
            f.write("TYPE F F F\n")
            f.write("COUNT 1 1 1\n")
            f.write(f"WIDTH {len(arr)}\n")
            f.write("HEIGHT 1\n")
            f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
            f.write(f"POINTS {len(arr)}\n")
            f.write("DATA ascii\n")
            for p in arr:
                f.write(f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n")

    write_xyz_only(ply_path, pcd_path, pcd)
    np.savetxt(txt_path, pts, fmt="%.6f")

    # Slightly transformed copy for registration
    T = np.eye(4)
    T[:3, :3] = o3d.geometry.get_rotation_matrix_from_xyz((0.12, -0.08, 0.05))
    T[:3, 3] = np.array([0.04, -0.03, 0.02])
    src = o3d.geometry.PointCloud(pcd)
    src.transform(T)
    write_xyz_only(DATA / "cloud_source.ply", DATA / "cloud_source.pcd", src)
    write_xyz_only(DATA / "cloud_target.ply", DATA / "cloud_target.pcd", pcd)

    depth_u16, color, depth_f = make_depth_and_rgbd()
    np.save(DATA / "depth_u16.npy", depth_u16)
    np.save(DATA / "color_rgb.npy", color)
    o3d.io.write_image(str(DATA / "depth.png"), o3d.geometry.Image(depth_u16))
    o3d.io.write_image(str(DATA / "color.png"), o3d.geometry.Image(color))

    # Binary mask covering central region
    h, w = depth_u16.shape
    mask = np.zeros((h, w), dtype=np.uint8)
    mask[h // 4 : 3 * h // 4, w // 4 : 3 * w // 4] = 255
    o3d.io.write_image(str(DATA / "mask.png"), o3d.geometry.Image(mask))
    np.save(DATA / "mask.npy", mask)

    meta = {
        "n_points": int(len(pts)),
        "bbox_min": pts.min(axis=0).tolist(),
        "bbox_max": pts.max(axis=0).tolist(),
        "seed": SEED,
        "depth_size": [int(w), int(h)],
        "intrinsics": {
            "width": w,
            "height": h,
            "fx": 250.0,
            "fy": 250.0,
            "cx": w / 2.0,
            "cy": h / 2.0,
        },
        "files": {
            "ply": str(ply_path.name),
            "pcd": str(pcd_path.name),
            "txt": str(txt_path.name),
            "source": "cloud_source.ply",
            "target": "cloud_target.ply",
            "depth": "depth.png",
            "color": "color.png",
            "mask": "mask.png",
        },
    }
    (DATA / "meta.json").write_text(json.dumps(meta, indent=2))
    print(f"Generated {len(pts)} points -> {DATA}")
    print(json.dumps(meta, indent=2))


if __name__ == "__main__":
    main()
