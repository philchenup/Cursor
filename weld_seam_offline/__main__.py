"""CLI: python -m weld_seam_offline input.ply --out-dir output/"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from .pipeline import detect_weld_seam
from .sample import write_sample_ply
from .visualize import render_result


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Detect a weld groove from a PLY point cloud and export the trajectory."
    )
    parser.add_argument("ply", nargs="?", help="Input PLY. Omit when using --make-sample.")
    parser.add_argument("--out-dir", default="output/weld_seam", help="Directory for CSV/PNG/NPZ.")
    parser.add_argument("--voxel-size", type=float, default=None, help="Downsample size. Auto if omitted.")
    parser.add_argument("--keep-ratio", type=float, default=0.05, help="Top feature fraction kept as groove seeds.")
    parser.add_argument("--max-depth", type=float, default=None, help="Optional Z cutoff (camera-frame clouds).")
    parser.add_argument("--tcp-offset", action="store_true", help="Apply the original uplift_z TCP offset.")
    parser.add_argument("--make-sample", action="store_true", help="Write a synthetic V-groove PLY and run on it.")
    parser.add_argument("--sample-path", default="data/sample_vgroove.ply", help="Where to write --make-sample.")
    parser.add_argument(
        "--pick",
        action="store_true",
        help="Open an Open3D window and pick points (Shift+left click). Needs a display.",
    )
    parser.add_argument("--point-size", type=float, default=6.0, help="Open3D point size when using --pick.")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    ply_path = Path(args.ply) if args.ply else None
    if args.make_sample:
        ply_path = write_sample_ply(args.sample_path)
        print(f"Wrote sample workpiece: {ply_path}")
    if ply_path is None:
        raise SystemExit("Provide a PLY path or pass --make-sample")

    if args.pick:
        from .pick import pick_from_ply

        picked = pick_from_ply(ply_path, voxel_size=args.voxel_size, point_size=args.point_size)
        out_dir = Path(args.out_dir)
        paths = picked.save(out_dir)
        print("Picked count:", len(picked.indices))
        print("Indices:", picked.indices.tolist())
        print("XYZ:")
        for i, xyz in zip(picked.indices, picked.xyz):
            print(f"  #{int(i):6d}  {xyz[0]: .6f} {xyz[1]: .6f} {xyz[2]: .6f}")
        print("CSV:", paths["csv"])
        return 0

    result = detect_weld_seam(
        ply_path,
        voxel_size=args.voxel_size,
        keep_ratio=args.keep_ratio,
        max_depth=args.max_depth,
        apply_tcp_offset=args.tcp_offset,
    )
    out_dir = Path(args.out_dir)
    paths = result.save(out_dir)
    image = render_result(result, out_dir / "trajectory.png", title=f"Weld seam: {ply_path.name}")

    summary = {
        "input": str(ply_path.resolve()),
        "points": int(len(result.cloud)),
        "groove_points": int(len(result.groove)),
        "waypoints": int(len(result.trajectory)),
        "voxel_size": result.voxel_size,
        "start": result.trajectory[0].tolist(),
        "end": result.trajectory[-1].tolist(),
        "length": float(np.sum(np.linalg.norm(np.diff(result.trajectory, axis=0), axis=1))),
        "files": {key: str(value) for key, value in paths.items()} | {"image": str(image)},
    }
    summary_path = out_dir / "summary.json"
    summary_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")

    print("Groove points:", summary["groove_points"])
    print("Waypoints:", summary["waypoints"])
    print("Trajectory length:", f"{summary['length']:.6f}")
    print("Start:", np.round(result.trajectory[0], 5).tolist())
    print("End:", np.round(result.trajectory[-1], 5).tolist())
    print("CSV:", paths["trajectory_csv"])
    print("Figure:", image)
    print("Summary:", summary_path)
    print("First 5 waypoints (x y z rx ry rz):")
    for row in result.poses[:5, :6]:
        print(" ", " ".join(f"{v: .5f}" for v in row))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
