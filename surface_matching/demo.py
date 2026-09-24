"""Reproduce PPF surface matching on a synthetic L-bracket."""

from __future__ import annotations

import argparse
import json

import numpy as np

from .geometry import make_transform, random_rotation, transform_error
from .pipeline import match_surface
from .shapes import make_scene, sample_corner_bracket


def run_demo(seed: int = 7) -> dict[str, float]:
    rng = np.random.default_rng(seed)
    model_points, model_normals = sample_corner_bracket(640, rng)
    ground_truth = make_transform(random_rotation(rng), rng.uniform(-0.4, 0.4, size=3))
    scene_points, scene_normals = make_scene(
        model_points,
        model_normals,
        ground_truth,
        rng,
        keep_ratio=0.72,
        noise=0.008,
        clutter=80,
    )
    result = match_surface(model_points, scene_points, model_normals, scene_normals)
    rotation_deg, translation = result.error_against(ground_truth)
    report = {
        "rotation_error_deg": rotation_deg,
        "translation_error": translation,
        "inlier_rmse": result.inlier_rmse,
        "inlier_ratio": result.inlier_ratio,
        "votes": result.votes,
        "hypotheses": len(result.hypotheses),
        "model_points": int(model_points.shape[0]),
        "scene_points": int(scene_points.shape[0]),
    }
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description="Run synthetic PPF surface matching.")
    parser.add_argument("--seed", type=int, default=7)
    args = parser.parse_args()
    report = run_demo(args.seed)
    print(json.dumps(report, indent=2))
    if report["rotation_error_deg"] > 5.0 or report["translation_error"] > 0.08:
        raise SystemExit("matching failed the synthetic accuracy gate")


if __name__ == "__main__":
    main()
