"""Point-to-point and point-to-plane ICP used to refine PPF hypotheses."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .geometry import apply_transform, as_points, make_transform, nearest_neighbor, normalize


@dataclass(frozen=True)
class ICPResult:
    transform: np.ndarray
    fitness: float
    rmse: float
    iterations: int


def point_to_point_icp(
    source: np.ndarray,
    target: np.ndarray,
    init: np.ndarray | None = None,
    max_iterations: int = 40,
    max_distance: float | None = None,
    tolerance: float = 1e-7,
) -> ICPResult:
    src = as_points(source)
    dst = as_points(target)
    transform = np.eye(4) if init is None else np.asarray(init, dtype=np.float64).copy()
    prev = np.inf
    rmse = np.inf
    fitness = 0.0
    if max_distance is None:
        max_distance = 0.2 * _extent(dst)

    iterations = 0
    for iterations in range(1, max_iterations + 1):
        moved = apply_transform(src, transform)
        indices, distances = nearest_neighbor(moved, dst)
        keep = distances <= max_distance
        if keep.sum() < 3:
            break
        rotation, translation = _kabsch(moved[keep], dst[indices[keep]])
        increment = make_transform(rotation, translation)
        transform = increment @ transform
        rmse = float(np.sqrt(np.mean(distances[keep] ** 2)))
        fitness = float(keep.mean())
        if abs(prev - rmse) < tolerance:
            break
        prev = rmse
    return ICPResult(transform=transform, fitness=fitness, rmse=rmse, iterations=iterations)


def point_to_plane_icp(
    source: np.ndarray,
    target: np.ndarray,
    target_normals: np.ndarray,
    init: np.ndarray | None = None,
    max_iterations: int = 40,
    max_distance: float | None = None,
    tolerance: float = 1e-7,
) -> ICPResult:
    src = as_points(source)
    dst = as_points(target)
    normals = normalize(np.asarray(target_normals, dtype=np.float64))
    transform = np.eye(4) if init is None else np.asarray(init, dtype=np.float64).copy()
    prev = np.inf
    rmse = np.inf
    fitness = 0.0
    if max_distance is None:
        max_distance = 0.2 * _extent(dst)

    iterations = 0
    for iterations in range(1, max_iterations + 1):
        moved = apply_transform(src, transform)
        indices, distances = nearest_neighbor(moved, dst)
        keep = distances <= max_distance
        if keep.sum() < 6:
            break
        increment = _point_to_plane_step(moved[keep], dst[indices[keep]], normals[indices[keep]])
        transform = increment @ transform
        residual = np.abs(np.sum((moved[keep] - dst[indices[keep]]) * normals[indices[keep]], axis=1))
        rmse = float(np.sqrt(np.mean(residual**2)))
        fitness = float(keep.mean())
        if abs(prev - rmse) < tolerance:
            break
        prev = rmse
    return ICPResult(transform=transform, fitness=fitness, rmse=rmse, iterations=iterations)


def _extent(points: np.ndarray) -> float:
    return float(np.linalg.norm(points.max(axis=0) - points.min(axis=0)))


def _kabsch(source: np.ndarray, target: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    src_mean = source.mean(axis=0)
    dst_mean = target.mean(axis=0)
    covariance = (source - src_mean).T @ (target - dst_mean)
    u, _, vt = np.linalg.svd(covariance)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0:
        vt[-1, :] *= -1
        rotation = vt.T @ u.T
    translation = dst_mean - rotation @ src_mean
    return rotation, translation


def _point_to_plane_step(source: np.ndarray, target: np.ndarray, normals: np.ndarray) -> np.ndarray:
    """Linearized point-to-plane increment (Low, 2004)."""
    cross = np.cross(source, normals)
    matrix = np.concatenate([cross, normals], axis=1)
    residual = -np.sum((source - target) * normals, axis=1)
    twist, *_ = np.linalg.lstsq(matrix, residual, rcond=None)
    rotation = _small_rotation(twist[:3])
    translation = twist[3:]
    return make_transform(rotation, translation)


def _small_rotation(omega: np.ndarray) -> np.ndarray:
    angle = float(np.linalg.norm(omega))
    if angle < 1e-12:
        return np.eye(3) + _skew(omega)
    axis = omega / angle
    skew = _skew(axis)
    return np.eye(3) + np.sin(angle) * skew + (1.0 - np.cos(angle)) * (skew @ skew)


def _skew(vector: np.ndarray) -> np.ndarray:
    x, y, z = vector
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]], dtype=np.float64)
