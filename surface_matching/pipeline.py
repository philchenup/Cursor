"""End-to-end surface matching: PPF coarse pose then point-to-plane ICP."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .geometry import estimate_normals, model_diameter, nearest_neighbor, transform_error
from .icp import ICPResult, point_to_plane_icp
from .ppf import PPFDetector, PoseHypothesis


@dataclass(frozen=True)
class MatchResult:
    transform: np.ndarray
    votes: float
    icp: ICPResult
    hypotheses: list[PoseHypothesis]
    inlier_rmse: float
    inlier_ratio: float

    def error_against(self, ground_truth: np.ndarray) -> tuple[float, float]:
        return transform_error(self.transform, ground_truth)


def match_surface(
    model_points: np.ndarray,
    scene_points: np.ndarray,
    model_normals: np.ndarray | None = None,
    scene_normals: np.ndarray | None = None,
    relative_sampling_step: float = 0.06,
    relative_distance_step: float = 0.05,
    scene_sample_step: float = 0.25,
    max_hypotheses: int = 8,
    refine: bool = True,
) -> MatchResult:
    """Match a model cloud to a scene cloud and return the best SE(3)."""
    if model_normals is None:
        model_normals = estimate_normals(model_points)
    if scene_normals is None:
        scene_normals = estimate_normals(scene_points)

    detector = PPFDetector(
        relative_sampling_step=relative_sampling_step,
        relative_distance_step=relative_distance_step,
    )
    detector.train(model_points, model_normals)
    hypotheses = detector.match(
        scene_points,
        scene_normals,
        scene_sample_step=scene_sample_step,
        max_hypotheses=max_hypotheses,
    )
    if not hypotheses:
        identity = np.eye(4)
        empty = ICPResult(transform=identity, fitness=0.0, rmse=np.inf, iterations=0)
        return MatchResult(identity, 0.0, empty, [], np.inf, 0.0)

    diameter = model_diameter(model_points)
    max_distance = 0.12 * diameter
    best: MatchResult | None = None
    best_score = np.inf
    for hypo in hypotheses:
        if refine:
            icp = point_to_plane_icp(
                model_points,
                scene_points,
                scene_normals,
                init=hypo.transform,
                max_distance=max_distance,
            )
            pose = icp.transform
        else:
            icp = ICPResult(transform=hypo.transform, fitness=0.0, rmse=np.inf, iterations=0)
            pose = hypo.transform
        rmse, ratio = _inlier_stats(model_points, scene_points, pose, max_distance)
        # Prefer high overlap, then low residual; votes break remaining ties.
        score = rmse / max(ratio, 1e-3) - 0.05 * hypo.votes
        if score < best_score:
            best_score = score
            best = MatchResult(pose, hypo.votes, icp, hypotheses, rmse, ratio)
    assert best is not None
    return best


def _inlier_stats(
    model_points: np.ndarray,
    scene_points: np.ndarray,
    transform: np.ndarray,
    max_distance: float,
) -> tuple[float, float]:
    from .geometry import apply_transform

    moved = apply_transform(model_points, transform)
    _, distances = nearest_neighbor(moved, scene_points)
    keep = distances <= max_distance
    if not np.any(keep):
        return float(np.sqrt(np.mean(distances**2))), 0.0
    return float(np.sqrt(np.mean(distances[keep] ** 2))), float(keep.mean())
