#!/usr/bin/env python3
"""Verify eye-to-hand 3D consistency uses T_end2base.inverse(), not T_end2base."""
import numpy as np


def T_from_Rt(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(t).ravel()
    return T


def invert_T(T):
    R = T[:3, :3]
    t = T[:3, 3]
    Ti = np.eye(4)
    Ti[:3, :3] = R.T
    Ti[:3, 3] = -R.T @ t
    return Ti


def rodrigues(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    K = np.array(
        [[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]]
    )
    return np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)


def random_pose(scale_t=400.0, seed=0):
    rng = np.random.RandomState(seed)
    R = rodrigues(rng.randn(3), rng.uniform(0.2, 1.0))
    t = rng.uniform(-scale_t, scale_t, size=3)
    return T_from_Rt(R, t)


def mean_abs_error(points_per_pose):
    """points_per_pose[j][i] = point j at pose i. Return global XYZ MAE."""
    abs_err = []
    for obs in points_per_pose:
        arr = np.asarray(obs)
        mean = arr.mean(axis=0)
        abs_err.extend(np.abs(arr - mean))
    return np.mean(abs_err, axis=0)


def eval_old_reuse(T_end2bases, T_cam2base, T_board2cams, objp):
    """Old bug: reuse EyeInHand kernel, left-multiply T_end2base."""
    pts = [[] for _ in objp]
    for Tend2base, Tboard2cam in zip(T_end2bases, T_board2cams):
        T = Tend2base @ T_cam2base @ Tboard2cam
        for j, p in enumerate(objp):
            pts[j].append((T @ np.append(p, 1.0))[:3])
    return mean_abs_error(pts)


def eval_fixed(T_end2bases, T_cam2base, T_board2cams, objp):
    """Fix: p_end = T_end2base.inverse() * T_cam2base * T_board2cam * p."""
    pts = [[] for _ in objp]
    for Tend2base, Tboard2cam in zip(T_end2bases, T_board2cams):
        T = invert_T(Tend2base) @ T_cam2base @ Tboard2cam
        for j, p in enumerate(objp):
            pts[j].append((T @ np.append(p, 1.0))[:3])
    return mean_abs_error(pts)


def main():
    T_cam2base = T_from_Rt(
        rodrigues([0.1, 0.8, 0.2], 0.6), [800.0, -200.0, 600.0]
    )
    T_board2end = T_from_Rt(rodrigues([0.3, 0.1, 0.9], 0.4), [20.0, -15.0, 80.0])

    xs, ys = np.meshgrid(np.arange(8) * 20.0, np.arange(5) * 20.0)
    objp = np.stack([xs.ravel(), ys.ravel(), np.zeros(xs.size)], axis=1)

    n = 8
    T_end2bases = [random_pose(300, seed=i) for i in range(n)]
    T_board2cams = [
        invert_T(T_cam2base) @ Te2b @ T_board2end for Te2b in T_end2bases
    ]

    mae_old = eval_old_reuse(T_end2bases, T_cam2base, T_board2cams, objp)
    mae_new = eval_fixed(T_end2bases, T_cam2base, T_board2cams, objp)

    print("old reuse (T_end2base * T_cam2base * T_board2cam) MAE:", mae_old)
    print("fixed     (T_end2base^{-1} * T_cam2base * T_board2cam) MAE:", mae_new)

    assert np.all(mae_old > 10.0), "old path should show large motion-scale error"
    assert np.all(mae_new < 1e-9), "fixed path should be numerically zero"
    print("OK")


if __name__ == "__main__":
    main()
