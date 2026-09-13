"""Synthetic V-groove workpiece used to exercise the offline pipeline."""

from __future__ import annotations

from pathlib import Path

import numpy as np

from .ply_io import save_ply


def make_vgroove_cloud(
    length: float = 0.24,
    plate_width: float = 0.08,
    groove_depth: float = 0.018,
    groove_angle_deg: float = 70.0,
    gap: float = 0.003,
    n_wall: int = 2800,
    n_plate: int = 3500,
    noise: float = 0.00035,
    curve_amp: float = 0.012,
    rng: np.random.Generator | None = None,
) -> np.ndarray:
    """Build a slightly curved V-groove plus two top plates, in metres."""
    rng = rng or np.random.default_rng(7)
    half = np.radians(groove_angle_deg) / 2.0
    slope = np.tan(half)
    half_open = groove_depth * slope + gap / 2.0

    def seam_offset(x: np.ndarray) -> np.ndarray:
        return curve_amp * np.sin(2.0 * np.pi * (x / length))

    def sample_wall(sign: float, count: int) -> np.ndarray:
        x = rng.uniform(0.0, length, count)
        t = rng.uniform(0.0, 1.0, count)
        y_local = sign * (gap / 2.0 + t * (half_open - gap / 2.0))
        z = (np.abs(y_local) - gap / 2.0) / max(slope, 1e-6)
        y = y_local + seam_offset(x)
        return np.column_stack([x, y, z])

    def sample_plate(sign: float, count: int) -> np.ndarray:
        x = rng.uniform(0.0, length, count)
        y_local = sign * rng.uniform(half_open, half_open + plate_width)
        y = y_local + seam_offset(x)
        z = np.full(count, groove_depth)
        return np.column_stack([x, y, z])

    left_wall = sample_wall(-1.0, n_wall)
    right_wall = sample_wall(+1.0, n_wall)
    left_plate = sample_plate(-1.0, n_plate)
    right_plate = sample_plate(+1.0, n_plate)
    cloud = np.vstack((left_wall, right_wall, left_plate, right_plate))
    cloud += rng.normal(0.0, noise, cloud.shape)
    return cloud


def write_sample_ply(path: str | Path, **kwargs) -> Path:
    path = Path(path)
    cloud = make_vgroove_cloud(**kwargs)
    save_ply(path, cloud)
    return path
