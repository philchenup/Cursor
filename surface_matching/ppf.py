"""Drost PPF voting with Hinterstoisser/Vidal robustness tweaks."""

from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass

import numpy as np

from .geometry import (
    EPS,
    angle_around_x,
    apply_rotation,
    apply_transform,
    as_points,
    invert_transform,
    make_transform,
    model_diameter,
    normalize,
    transform_to_x_axis,
    voxel_downsample,
)


@dataclass(frozen=True)
class PoseHypothesis:
    transform: np.ndarray
    votes: float
    model_index: int
    scene_index: int
    alpha: float


class PPFDetector:
    """Hash oriented point pairs and recover coarse SE(3) hypotheses.

    This follows Drost et al., CVPR 2010, with two widely used fixes:

    * lookup of neighboring quantized bins (Vidal / Hinterstoisser spreading)
    * at most one vote per (feature bin, alpha bin) per reference point
      (Hinterstoisser, ECCV 2016), which cuts planar clutter
    """

    def __init__(
        self,
        relative_sampling_step: float = 0.05,
        relative_distance_step: float = 0.05,
        angle_step_deg: float = 12.0,
        alpha_bins: int = 30,
        neighbor_bins: bool = True,
        vote_once: bool = True,
    ) -> None:
        if relative_sampling_step <= 0 or relative_distance_step <= 0:
            raise ValueError("sampling and distance steps must be positive")
        self.relative_sampling_step = float(relative_sampling_step)
        self.relative_distance_step = float(relative_distance_step)
        self.angle_step = float(np.radians(angle_step_deg))
        self.alpha_bins = int(alpha_bins)
        self.neighbor_bins = bool(neighbor_bins)
        self.vote_once = bool(vote_once)
        self.model_points = np.zeros((0, 3), dtype=np.float64)
        self.model_normals = np.zeros((0, 3), dtype=np.float64)
        self.model_frames: list[tuple[np.ndarray, np.ndarray]] = []
        self.hash_table: dict[tuple[int, int, int, int], list[tuple[int, float]]] = {}
        self.distance_step = 1.0
        self.diameter = 1.0

    def train(self, points: np.ndarray, normals: np.ndarray) -> None:
        pts = as_points(points)
        nrm = normalize(np.asarray(normals, dtype=np.float64))
        if pts.shape[0] != nrm.shape[0]:
            raise ValueError("points and normals must have the same length")
        self.diameter = max(model_diameter(pts), EPS)
        leaf = self.relative_sampling_step * self.diameter
        self.distance_step = self.relative_distance_step * self.diameter
        self.model_points, self.model_normals = voxel_downsample(pts, nrm, leaf)
        self.model_frames = [
            transform_to_x_axis(point, normal)
            for point, normal in zip(self.model_points, self.model_normals)
        ]
        self.hash_table = defaultdict(list)
        count = self.model_points.shape[0]
        for ref in range(count):
            rotation, translation = self.model_frames[ref]
            for other in range(count):
                if other == ref:
                    continue
                feature = _point_pair_feature(
                    self.model_points[ref],
                    self.model_normals[ref],
                    self.model_points[other],
                    self.model_normals[other],
                )
                if feature is None:
                    continue
                other_local = rotation @ self.model_points[other] + translation
                if other_local[1] ** 2 + other_local[2] ** 2 < 1e-10:
                    continue
                key = self._quantize(feature)
                self.hash_table[key].append((ref, angle_around_x(other_local)))

    def match(
        self,
        scene_points: np.ndarray,
        scene_normals: np.ndarray,
        scene_sample_step: float = 0.2,
        min_votes: float = 3.0,
        max_hypotheses: int = 16,
    ) -> list[PoseHypothesis]:
        if self.model_points.shape[0] == 0:
            raise RuntimeError("call train() before match()")
        pts = as_points(scene_points)
        nrm = normalize(np.asarray(scene_normals, dtype=np.float64))
        leaf = self.relative_sampling_step * self.diameter
        scene_pts, scene_nrm = voxel_downsample(pts, nrm, leaf)
        count = scene_pts.shape[0]
        if count < 3:
            return []
        stride = max(1, int(round(1.0 / max(scene_sample_step, 1e-6))))
        references = range(0, count, stride)
        raw: list[PoseHypothesis] = []
        for scene_ref in references:
            rotation_s, translation_s = transform_to_x_axis(scene_pts[scene_ref], scene_nrm[scene_ref])
            accumulator = np.zeros((self.model_points.shape[0], self.alpha_bins), dtype=np.float64)
            seen: set[tuple[tuple[int, int, int, int], int]] = set()
            for other in range(count):
                if other == scene_ref:
                    continue
                feature = _point_pair_feature(
                    scene_pts[scene_ref],
                    scene_nrm[scene_ref],
                    scene_pts[other],
                    scene_nrm[other],
                )
                if feature is None:
                    continue
                other_local = rotation_s @ scene_pts[other] + translation_s
                if other_local[1] ** 2 + other_local[2] ** 2 < 1e-10:
                    continue
                alpha_s = angle_around_x(other_local)
                for key in self._lookup_keys(feature):
                    entries = self.hash_table.get(key)
                    if not entries:
                        continue
                    for model_ref, alpha_m in entries:
                        alpha = _wrap_pi(alpha_s - alpha_m)
                        bin_index = self._alpha_bin(alpha)
                        vote_id = (key, bin_index)
                        if self.vote_once and vote_id in seen:
                            continue
                        if self.vote_once:
                            seen.add(vote_id)
                        accumulator[model_ref, bin_index] += 1.0
            peaks = _top_peaks(accumulator, min_votes=min_votes, limit=4)
            scene_to_canonical = make_transform(rotation_s, translation_s)
            for model_ref, bin_index, votes in peaks:
                alpha = self._bin_center(bin_index)
                transform = _pose_from_vote(self.model_frames[model_ref], scene_to_canonical, alpha)
                raw.append(
                    PoseHypothesis(
                        transform=transform,
                        votes=float(votes),
                        model_index=int(model_ref),
                        scene_index=int(scene_ref),
                        alpha=float(alpha),
                    )
                )
        clustered = cluster_poses(
            raw,
            translation_thresh=0.1 * self.diameter,
            rotation_thresh_deg=15.0,
        )
        clustered.sort(key=lambda item: item.votes, reverse=True)
        return clustered[:max_hypotheses]

    def _quantize(self, feature: np.ndarray) -> tuple[int, int, int, int]:
        dist, a1, a2, a3 = feature
        return (
            int(np.floor(dist / self.distance_step)),
            int(np.floor(a1 / self.angle_step)),
            int(np.floor(a2 / self.angle_step)),
            int(np.floor(a3 / self.angle_step)),
        )

    def _lookup_keys(self, feature: np.ndarray) -> list[tuple[int, int, int, int]]:
        center = self._quantize(feature)
        if not self.neighbor_bins:
            return [center]
        keys = [center]
        for axis in range(4):
            for delta in (-1, 1):
                neighbor = list(center)
                neighbor[axis] += delta
                keys.append(tuple(neighbor))
        return keys

    def _alpha_bin(self, alpha: float) -> int:
        wrapped = (alpha + np.pi) / (2.0 * np.pi)
        return int(np.floor(wrapped * self.alpha_bins)) % self.alpha_bins

    def _bin_center(self, bin_index: int) -> float:
        return -np.pi + (bin_index + 0.5) * (2.0 * np.pi / self.alpha_bins)


def cluster_poses(
    hypotheses: list[PoseHypothesis],
    translation_thresh: float,
    rotation_thresh_deg: float,
) -> list[PoseHypothesis]:
    if not hypotheses:
        return []
    ordered = sorted(hypotheses, key=lambda item: item.votes, reverse=True)
    clusters: list[list[PoseHypothesis]] = []
    for hypo in ordered:
        assigned = False
        for cluster in clusters:
            if _pose_close(hypo.transform, cluster[0].transform, translation_thresh, rotation_thresh_deg):
                cluster.append(hypo)
                assigned = True
                break
        if not assigned:
            clusters.append([hypo])
    merged: list[PoseHypothesis] = []
    for cluster in clusters:
        rotations = [item.transform[:3, :3] for item in cluster]
        translations = [item.transform[:3, 3] for item in cluster]
        weights = np.array([item.votes for item in cluster], dtype=np.float64)
        rotation = _average_rotations(rotations, weights)
        translation = np.average(np.stack(translations), axis=0, weights=weights)
        seed = cluster[0]
        merged.append(
            PoseHypothesis(
                transform=make_transform(rotation, translation),
                votes=float(weights.sum()),
                model_index=seed.model_index,
                scene_index=seed.scene_index,
                alpha=seed.alpha,
            )
        )
    return merged


def _point_pair_feature(
    p1: np.ndarray,
    n1: np.ndarray,
    p2: np.ndarray,
    n2: np.ndarray,
) -> np.ndarray | None:
    delta = p2 - p1
    dist = float(np.linalg.norm(delta))
    if dist < EPS:
        return None
    direction = delta / dist
    return np.array(
        [
            dist,
            _safe_angle(n1, direction),
            _safe_angle(n2, direction),
            _safe_angle(n1, n2),
        ],
        dtype=np.float64,
    )


def _safe_angle(left: np.ndarray, right: np.ndarray) -> float:
    return float(np.arccos(np.clip(left @ right, -1.0, 1.0)))


def _wrap_pi(angle: float) -> float:
    return float((angle + np.pi) % (2.0 * np.pi) - np.pi)


def _rotation_x(alpha: float) -> np.ndarray:
    cosine, sine = np.cos(alpha), np.sin(alpha)
    return np.array(
        [[1.0, 0.0, 0.0], [0.0, cosine, -sine], [0.0, sine, cosine]],
        dtype=np.float64,
    )


def _pose_from_vote(
    model_frame: tuple[np.ndarray, np.ndarray],
    scene_to_canonical: np.ndarray,
    alpha: float,
) -> np.ndarray:
    model_to_canonical = make_transform(*model_frame)
    return invert_transform(scene_to_canonical) @ make_transform(_rotation_x(alpha), np.zeros(3)) @ model_to_canonical


def _top_peaks(
    accumulator: np.ndarray,
    min_votes: float,
    limit: int,
) -> list[tuple[int, int, float]]:
    flat = accumulator.ravel()
    if flat.size == 0:
        return []
    keep = min(limit, flat.size)
    candidates = np.argpartition(flat, -keep)[-keep:]
    peaks: list[tuple[int, int, float]] = []
    columns = accumulator.shape[1]
    for index in candidates:
        votes = float(flat[index])
        if votes < min_votes:
            continue
        model_ref, bin_index = divmod(int(index), columns)
        peaks.append((model_ref, bin_index, votes))
    peaks.sort(key=lambda item: item[2], reverse=True)
    return peaks


def _pose_close(
    left: np.ndarray,
    right: np.ndarray,
    translation_thresh: float,
    rotation_thresh_deg: float,
) -> bool:
    delta = invert_transform(right) @ left
    angle = np.degrees(np.arccos(np.clip((np.trace(delta[:3, :3]) - 1.0) * 0.5, -1.0, 1.0)))
    return float(np.linalg.norm(delta[:3, 3])) <= translation_thresh and angle <= rotation_thresh_deg


def _average_rotations(rotations: list[np.ndarray], weights: np.ndarray) -> np.ndarray:
    quats = np.stack([_rotation_to_quat(item) for item in rotations])
    if np.dot(quats[0], quats.T).min() < 0:
        signs = np.sign(quats @ quats[0])
        signs[signs == 0] = 1.0
        quats = quats * signs[:, None]
    mean = normalize(weights @ quats)
    return _quat_to_rotation(mean)


def _rotation_to_quat(rotation: np.ndarray) -> np.ndarray:
    trace = np.trace(rotation)
    if trace > 0:
        scale = 0.5 / np.sqrt(trace + 1.0)
        return np.array(
            [
                0.25 / scale,
                (rotation[2, 1] - rotation[1, 2]) * scale,
                (rotation[0, 2] - rotation[2, 0]) * scale,
                (rotation[1, 0] - rotation[0, 1]) * scale,
            ]
        )
    if rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        scale = 2.0 * np.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2])
        return np.array(
            [
                (rotation[2, 1] - rotation[1, 2]) / scale,
                0.25 * scale,
                (rotation[0, 1] + rotation[1, 0]) / scale,
                (rotation[0, 2] + rotation[2, 0]) / scale,
            ]
        )
    if rotation[1, 1] > rotation[2, 2]:
        scale = 2.0 * np.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2])
        return np.array(
            [
                (rotation[0, 2] - rotation[2, 0]) / scale,
                (rotation[0, 1] + rotation[1, 0]) / scale,
                0.25 * scale,
                (rotation[1, 2] + rotation[2, 1]) / scale,
            ]
        )
    scale = 2.0 * np.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1])
    return np.array(
        [
            (rotation[1, 0] - rotation[0, 1]) / scale,
            (rotation[0, 2] + rotation[2, 0]) / scale,
            (rotation[1, 2] + rotation[2, 1]) / scale,
            0.25 * scale,
        ]
    )


def _quat_to_rotation(quat: np.ndarray) -> np.ndarray:
    w, x, y, z = quat
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def transform_cloud(
    points: np.ndarray,
    normals: np.ndarray,
    transform: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    return apply_transform(points, transform), apply_rotation(normals, transform)
