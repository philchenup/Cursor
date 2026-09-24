"""Rigid transforms, normals, and pairing utilities for surface matching."""

from __future__ import annotations

import numpy as np

EPS = 1e-12


def as_points(points: np.ndarray) -> np.ndarray:
    pts = np.asarray(points, dtype=np.float64)
    if pts.ndim != 2 or pts.shape[1] != 3:
        raise ValueError("points must have shape (N, 3)")
    return pts


def normalize(vectors: np.ndarray, axis: int = -1) -> np.ndarray:
    vec = np.asarray(vectors, dtype=np.float64)
    norm = np.linalg.norm(vec, axis=axis, keepdims=True)
    return vec / np.maximum(norm, EPS)


def make_transform(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = np.asarray(rotation, dtype=np.float64)
    transform[:3, 3] = np.asarray(translation, dtype=np.float64).reshape(3)
    return transform


def invert_transform(transform: np.ndarray) -> np.ndarray:
    rotation = transform[:3, :3]
    translation = transform[:3, 3]
    inverse = np.eye(4, dtype=np.float64)
    inverse[:3, :3] = rotation.T
    inverse[:3, 3] = -rotation.T @ translation
    return inverse


def apply_transform(points: np.ndarray, transform: np.ndarray) -> np.ndarray:
    pts = as_points(points)
    rotation = transform[:3, :3]
    translation = transform[:3, 3]
    return pts @ rotation.T + translation


def apply_rotation(normals: np.ndarray, transform: np.ndarray) -> np.ndarray:
    return normalize(np.asarray(normals, dtype=np.float64) @ transform[:3, :3].T)


def random_rotation(rng: np.random.Generator) -> np.ndarray:
    """Uniform random rotation via QR of a Gaussian matrix with det +1."""
    matrix = rng.normal(size=(3, 3))
    rotation, _ = np.linalg.qr(matrix)
    if np.linalg.det(rotation) < 0:
        rotation[:, 0] *= -1
    return rotation


def rotation_angle_deg(rotation: np.ndarray) -> float:
    trace = np.clip((np.trace(rotation) - 1.0) * 0.5, -1.0, 1.0)
    return float(np.degrees(np.arccos(trace)))


def transform_error(estimated: np.ndarray, ground_truth: np.ndarray) -> tuple[float, float]:
    delta = invert_transform(ground_truth) @ estimated
    return rotation_angle_deg(delta[:3, :3]), float(np.linalg.norm(delta[:3, 3]))


def rotation_aligning(source: np.ndarray, target: np.ndarray) -> np.ndarray:
    """Rodrigues rotation that maps unit vector `source` onto `target`."""
    src = normalize(source)
    dst = normalize(target)
    cosine = float(np.clip(src @ dst, -1.0, 1.0))
    if cosine > 1.0 - 1e-10:
        return np.eye(3, dtype=np.float64)
    if cosine < -1.0 + 1e-10:
        axis = np.array([1.0, 0.0, 0.0], dtype=np.float64)
        if abs(src[0]) > 0.9:
            axis = np.array([0.0, 1.0, 0.0], dtype=np.float64)
        axis = normalize(np.cross(src, axis))
        return _rodrigues(axis, np.pi)
    axis = np.cross(src, dst)
    sine = float(np.linalg.norm(axis))
    skew = _skew(axis)
    return np.eye(3) + skew + skew @ skew * ((1.0 - cosine) / (sine * sine))


def transform_to_x_axis(point: np.ndarray, normal: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Map `point` to the origin and `normal` to +X."""
    rotation = rotation_aligning(normal, np.array([1.0, 0.0, 0.0]))
    translation = -rotation @ point
    return rotation, translation


def angle_around_x(point: np.ndarray) -> float:
    return float(np.arctan2(point[2], point[1]))


def estimate_normals(points: np.ndarray, k: int = 16) -> np.ndarray:
    """PCA normals from k nearest neighbors. Signs are flipped to face the centroid."""
    pts = as_points(points)
    count = pts.shape[0]
    if count < 4:
        raise ValueError("need at least 4 points to estimate normals")
    neighbors = min(k, count - 1)
    normals = np.zeros_like(pts)
    centroid = pts.mean(axis=0)
    for index, point in enumerate(pts):
        distances = np.linalg.norm(pts - point, axis=1)
        distances[index] = np.inf
        nearest = pts[np.argpartition(distances, neighbors)[:neighbors]]
        cov = np.cov(nearest - point, rowvar=False)
        _, vectors = np.linalg.eigh(cov)
        normal = vectors[:, 0]
        if (centroid - point) @ normal > 0:
            normal = -normal
        normals[index] = normal
    return normalize(normals)


def voxel_downsample(points: np.ndarray, normals: np.ndarray, leaf: float) -> tuple[np.ndarray, np.ndarray]:
    pts = as_points(points)
    nrm = normalize(np.asarray(normals, dtype=np.float64))
    if leaf <= 0:
        return pts, nrm
    keys = np.floor(pts / leaf).astype(np.int64)
    buckets: dict[tuple[int, int, int], list[int]] = {}
    for idx, key in enumerate(map(tuple, keys)):
        buckets.setdefault(key, []).append(idx)
    kept_points = []
    kept_normals = []
    for indices in buckets.values():
        subset = pts[indices]
        center = subset.mean(axis=0)
        nearest = indices[int(np.argmin(np.linalg.norm(subset - center, axis=1)))]
        kept_points.append(pts[nearest])
        kept_normals.append(nrm[nearest])
    return np.stack(kept_points), np.stack(kept_normals)


def model_diameter(points: np.ndarray) -> float:
    pts = as_points(points)
    return float(np.linalg.norm(pts.max(axis=0) - pts.min(axis=0)))


def nearest_neighbor(source: np.ndarray, target: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Brute-force nearest neighbor. Fine for a few thousand points."""
    src = as_points(source)
    dst = as_points(target)
    distances = np.linalg.norm(src[:, None, :] - dst[None, :, :], axis=2)
    indices = distances.argmin(axis=1)
    return indices, distances[np.arange(src.shape[0]), indices]


def _skew(vector: np.ndarray) -> np.ndarray:
    x, y, z = vector
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]], dtype=np.float64)


def _rodrigues(axis: np.ndarray, angle: float) -> np.ndarray:
    skew = _skew(normalize(axis))
    return np.eye(3) + np.sin(angle) * skew + (1.0 - np.cos(angle)) * (skew @ skew)
