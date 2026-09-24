from __future__ import annotations

import numpy as np
import pytest

from surface_matching.geometry import (
    apply_transform,
    estimate_normals,
    invert_transform,
    make_transform,
    random_rotation,
    rotation_aligning,
    transform_error,
)
from surface_matching.icp import point_to_plane_icp, point_to_point_icp
from surface_matching.pipeline import match_surface
from surface_matching.shapes import make_scene, sample_corner_bracket


def test_rotation_aligning_maps_vector_to_x():
    rng = np.random.default_rng(1)
    vector = rng.normal(size=3)
    vector /= np.linalg.norm(vector)
    rotation = rotation_aligning(vector, np.array([1.0, 0.0, 0.0]))
    aligned = rotation @ vector
    np.testing.assert_allclose(aligned, [1.0, 0.0, 0.0], atol=1e-8)


def test_invert_transform_roundtrip():
    rng = np.random.default_rng(2)
    transform = make_transform(random_rotation(rng), rng.normal(size=3))
    recovered = invert_transform(transform) @ transform
    np.testing.assert_allclose(recovered, np.eye(4), atol=1e-9)


def test_point_to_point_icp_refines_small_motion():
    rng = np.random.default_rng(3)
    model, _ = sample_corner_bracket(300, rng)
    truth = make_transform(
        rotation_aligning(np.array([1.0, 0.0, 0.0]), np.array([0.98, 0.15, 0.1])),
        np.array([0.04, -0.03, 0.02]),
    )
    scene = apply_transform(model, truth)
    result = point_to_point_icp(model, scene, init=np.eye(4))
    rot, trans = transform_error(result.transform, truth)
    assert rot < 1.0
    assert trans < 0.02


def test_ppf_recovers_large_unknown_pose():
    rng = np.random.default_rng(11)
    model_points, model_normals = sample_corner_bracket(560, rng)
    truth = make_transform(random_rotation(rng), rng.uniform(-0.35, 0.35, size=3))
    scene_points, scene_normals = make_scene(
        model_points,
        model_normals,
        truth,
        rng,
        keep_ratio=0.75,
        noise=0.006,
        clutter=40,
    )
    result = match_surface(model_points, scene_points, model_normals, scene_normals)
    rot, trans = result.error_against(truth)
    assert result.hypotheses, "PPF should emit at least one hypothesis"
    assert rot < 4.0
    assert trans < 0.06
    assert result.inlier_ratio > 0.45


def test_ppf_succeeds_when_icp_from_identity_fails():
    rng = np.random.default_rng(19)
    model_points, model_normals = sample_corner_bracket(520, rng)
    # A large rotation with no overlap-friendly init.
    truth = make_transform(random_rotation(rng), np.array([0.5, -0.4, 0.3]))
    scene_points, scene_normals = make_scene(
        model_points,
        model_normals,
        truth,
        rng,
        keep_ratio=1.0,
        noise=0.0,
        clutter=0,
    )
    naive = point_to_plane_icp(model_points, scene_points, scene_normals, init=np.eye(4))
    naive_rot, _ = transform_error(naive.transform, truth)
    result = match_surface(model_points, scene_points, model_normals, scene_normals)
    rot, trans = result.error_against(truth)
    assert naive_rot > 15.0
    assert rot < 4.0
    assert trans < 0.06


def test_normals_can_be_estimated_from_points_only():
    rng = np.random.default_rng(5)
    model_points, _ = sample_corner_bracket(400, rng)
    normals = estimate_normals(model_points)
    assert normals.shape == model_points.shape
    assert np.allclose(np.linalg.norm(normals, axis=1), 1.0, atol=1e-6)


@pytest.mark.parametrize("seed", [3, 8, 21])
def test_matching_is_seed_stable(seed: int):
    rng = np.random.default_rng(seed)
    model_points, model_normals = sample_corner_bracket(500, rng)
    truth = make_transform(random_rotation(rng), rng.uniform(-0.25, 0.25, size=3))
    scene_points, scene_normals = make_scene(
        model_points,
        model_normals,
        truth,
        rng,
        keep_ratio=0.8,
        noise=0.004,
        clutter=20,
    )
    result = match_surface(model_points, scene_points, model_normals, scene_normals)
    rot, trans = result.error_against(truth)
    assert rot < 5.0
    assert trans < 0.07
