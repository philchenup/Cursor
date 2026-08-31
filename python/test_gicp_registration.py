"""Synthetic GICP regression test. Run: python test_gicp_registration.py"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from gicp_registration import make_synthetic_pair, register_gicp, register_gicp_multiscale


def test_single_scale_recovers_small_offset() -> None:
    source, target, gt = make_synthetic_pair(
        n_points=3000,
        translation=(0.04, -0.03, 0.02),
        yaw_deg=6.0,
        seed=1,
    )
    result = register_gicp(source, target, voxel_size=0.03, max_iteration=80)
    trans_err = np.linalg.norm(result.transformation[:3, 3] - gt[:3, 3])
    assert result.fitness > 0.4, result.fitness
    assert trans_err < 0.03, trans_err


def test_multiscale_recovers_larger_offset() -> None:
    source, target, gt = make_synthetic_pair(
        n_points=4000,
        translation=(0.12, -0.08, 0.05),
        yaw_deg=12.0,
        seed=0,
    )
    result = register_gicp_multiscale(
        source, target, voxel_sizes=(0.08, 0.04, 0.02), max_iteration=64
    )
    trans_err = np.linalg.norm(result.transformation[:3, 3] - gt[:3, 3])
    rot_err = np.linalg.norm(result.transformation[:3, :3] - gt[:3, :3], ord="fro")
    assert result.fitness > 0.5, result.fitness
    assert trans_err < 0.02, trans_err
    assert rot_err < 0.15, rot_err


if __name__ == "__main__":
    test_single_scale_recovers_small_offset()
    test_multiscale_recovers_larger_offset()
    print("PASS")
