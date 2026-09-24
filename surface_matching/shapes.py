"""Asymmetric synthetic surfaces used to reproduce matching without extra data."""

from __future__ import annotations

import numpy as np

from .geometry import normalize


def sample_corner_bracket(count: int = 700, rng: np.random.Generator | None = None) -> tuple[np.ndarray, np.ndarray]:
    """Three orthogonal plates forming an L-bracket with unequal sizes."""
    rng = rng or np.random.default_rng(0)
    weights = np.array([2.0 * 1.1, 2.0 * 0.7, 1.1 * 0.7], dtype=np.float64)
    weights /= weights.sum()
    counts = rng.multinomial(count, weights)

    xy_points, xy_normals = _sample_plate(counts[0], (2.0, 1.1), "z", rng)
    xz_points, xz_normals = _sample_plate(counts[1], (2.0, 0.7), "y", rng)
    yz_points, yz_normals = _sample_plate(counts[2], (1.1, 0.7), "x", rng)

    # Shift plates so they share the origin corner and stay unique.
    xz_points[:, 1] = 0.0
    yz_points[:, 0] = 0.0

    # Add a raised cylindrical boss so the part is not a pure corner.
    boss_count = max(40, count // 12)
    theta = rng.uniform(0, 2 * np.pi, size=boss_count)
    height = rng.uniform(0.15, 0.55, size=boss_count)
    radius = 0.22
    boss = np.column_stack(
        [
            1.35 + radius * np.cos(theta),
            0.45 + radius * np.sin(theta),
            height,
        ]
    )
    boss_normals = normalize(np.column_stack([np.cos(theta), np.sin(theta), np.zeros(boss_count)]))

    points = np.concatenate([xy_points, xz_points, yz_points, boss], axis=0)
    normals = np.concatenate([xy_normals, xz_normals, yz_normals, boss_normals], axis=0)
    return points, normals


def make_scene(
    model_points: np.ndarray,
    model_normals: np.ndarray,
    transform: np.ndarray,
    rng: np.random.Generator,
    keep_ratio: float = 0.7,
    noise: float = 0.0,
    clutter: int = 0,
) -> tuple[np.ndarray, np.ndarray]:
    """Apply a pose, drop a subset, add noise and random clutter points."""
    from .geometry import apply_rotation, apply_transform

    points = apply_transform(model_points, transform)
    normals = apply_rotation(model_normals, transform)
    if keep_ratio < 1.0:
        # Keep the more visible half along a random camera axis, then subsample.
        axis = rng.normal(size=3)
        axis /= np.linalg.norm(axis)
        score = points @ axis
        cutoff = np.quantile(score, 1.0 - keep_ratio)
        visible = score >= cutoff
        points = points[visible]
        normals = normals[visible]
    if noise > 0:
        points = points + rng.normal(scale=noise, size=points.shape)
    if clutter > 0:
        low = points.min(axis=0) - 0.4
        high = points.max(axis=0) + 0.8
        extra = rng.uniform(low, high, size=(clutter, 3))
        extra_normals = normalize(rng.normal(size=(clutter, 3)))
        points = np.concatenate([points, extra], axis=0)
        normals = np.concatenate([normals, extra_normals], axis=0)
    return points, normals


def _sample_plate(
    count: int,
    size: tuple[float, float],
    axis: str,
    rng: np.random.Generator,
) -> tuple[np.ndarray, np.ndarray]:
    width, height = size
    u = rng.uniform(0.0, width, size=count)
    v = rng.uniform(0.0, height, size=count)
    points = np.zeros((count, 3), dtype=np.float64)
    normals = np.zeros((count, 3), dtype=np.float64)
    if axis == "z":
        points[:, 0] = u
        points[:, 1] = v
        normals[:, 2] = 1.0
    elif axis == "y":
        points[:, 0] = u
        points[:, 2] = v
        normals[:, 1] = 1.0
    else:
        points[:, 1] = u
        points[:, 2] = v
        normals[:, 0] = 1.0
    return points, normals
