#!/usr/bin/env python3
"""Verify eye-to-hand 3D consistency and first-frame skip matching updateTable."""
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


def collect_points(T_list, objp):
    """pts[j][i] = point j at pose i."""
    pts = [[] for _ in objp]
    for T in T_list:
        for j, p in enumerate(objp):
            pts[j].append((T @ np.append(p, 1.0))[:3])
    return pts


def mean_abs_error(points_per_pose, skip_first=False):
    """points_per_pose[j][i] = point j at pose i. Return global XYZ MAE."""
    start = 1 if skip_first else 0
    abs_err = []
    for obs in points_per_pose:
        arr = np.asarray(obs)
        mean = arr[start:].mean(axis=0)
        abs_err.extend(np.abs(arr[start:] - mean))
    return np.mean(abs_err, axis=0)


def per_pose_mean_abs_packed(points_per_pose):
    """Match C++: skip pose 0 in the reference, pack errors for poses 1..n-1."""
    n_pose = len(points_per_pose[0])
    n_pts = len(points_per_pose)
    packed = []
    for i in range(1, n_pose):
        acc = np.zeros(3)
        for obs in points_per_pose:
            arr = np.asarray(obs)
            ref = arr[1:].mean(axis=0)
            acc += np.abs(arr[i] - ref)
        packed.append(acc / n_pts)
    return packed


def simulate_update_table(valid_index, per_pose_xyz):
    """Mirror HandEyeCalib::updateTable mapping. Returns display strings per row."""
    cells = []
    valid_cnt = 0
    for is_valid in valid_index:
        if is_valid and valid_cnt < len(per_pose_xyz):
            e = per_pose_xyz[valid_cnt]
            cells.append(tuple(f"{v:.4f}" for v in e))
            valid_cnt += 1
        else:
            cells.append(("-", "-", "-"))
    return cells


def eval_old_reuse(T_end2bases, T_cam2base, T_board2cams, objp):
    """Old bug: reuse EyeInHand kernel, left-multiply T_end2base."""
    Ts = [Te2b @ T_cam2base @ Tboard2cam
          for Te2b, Tboard2cam in zip(T_end2bases, T_board2cams)]
    return mean_abs_error(collect_points(Ts, objp), skip_first=True)


def eval_fixed(T_end2bases, T_cam2base, T_board2cams, objp):
    """Fix: p_end = T_end2base.inverse() * T_cam2base * T_board2cam * p."""
    Ts = [invert_T(Te2b) @ T_cam2base @ Tboard2cam
          for Te2b, Tboard2cam in zip(T_end2bases, T_board2cams)]
    return mean_abs_error(collect_points(Ts, objp), skip_first=True)


def eval_in_hand(T_end2bases, T_cam2end, T_board2cams, objp):
    """Eye-in-hand: p_base = T_end2base * T_cam2end * T_board2cam * p."""
    Ts = [Te2b @ T_cam2end @ Tboard2cam
          for Te2b, Tboard2cam in zip(T_end2bases, T_board2cams)]
    return mean_abs_error(collect_points(Ts, objp), skip_first=True)


def main():
    T_cam2base = T_from_Rt(
        rodrigues([0.1, 0.8, 0.2], 0.6), [800.0, -200.0, 600.0]
    )
    T_board2end = T_from_Rt(rodrigues([0.3, 0.1, 0.9], 0.4), [20.0, -15.0, 80.0])
    T_cam2end = T_from_Rt(rodrigues([0.2, 0.4, 0.7], 0.5), [30.0, 10.0, 90.0])
    T_board2base = T_from_Rt(rodrigues([0.0, 0.1, 0.2], 0.3), [400.0, 50.0, 200.0])

    xs, ys = np.meshgrid(np.arange(8) * 20.0, np.arange(5) * 20.0)
    objp = np.stack([xs.ravel(), ys.ravel(), np.zeros(xs.size)], axis=1)

    n = 8
    T_end2bases = [random_pose(300, seed=i) for i in range(n)]
    T_board2cams_on = [
        invert_T(T_cam2base) @ Te2b @ T_board2end for Te2b in T_end2bases
    ]
    T_board2cams_in = [
        invert_T(T_cam2end) @ invert_T(Te2b) @ T_board2base for Te2b in T_end2bases
    ]

    mae_old = eval_old_reuse(T_end2bases, T_cam2base, T_board2cams_on, objp)
    mae_on = eval_fixed(T_end2bases, T_cam2base, T_board2cams_on, objp)
    mae_in = eval_in_hand(T_end2bases, T_cam2end, T_board2cams_in, objp)

    print("old reuse MAE:", mae_old)
    print("eye-on-hand MAE (skip first):", mae_on)
    print("eye-in-hand MAE (skip first):", mae_in)

    assert np.all(mae_old > 10.0), "old path should show large motion-scale error"
    assert np.all(mae_on < 1e-9), "eye-on-hand skip-first path should be numerically zero"
    assert np.all(mae_in < 1e-9), "eye-in-hand skip-first path should be numerically zero"

    # Pose 0 outlier must not leak into remaining errors (same as unchecking row 0).
    Ts_on = [invert_T(Te2b) @ T_cam2base @ Tb2c
             for Te2b, Tb2c in zip(T_end2bases, T_board2cams_on)]
    pts = collect_points(Ts_on, objp)
    for j in range(len(pts)):
        pts[j][0] = np.asarray(pts[j][0]) + np.array([800.0, 50.0, 0.0])

    packed = per_pose_mean_abs_packed(pts)
    assert len(packed) == n - 1
    for e in packed:
        assert np.all(e < 1e-9), "packed pose errors must ignore the first-frame outlier"

    mae_skip = mean_abs_error(pts, skip_first=True)
    mae_all = mean_abs_error(pts, skip_first=False)
    assert np.all(mae_skip < 1e-9)
    assert mae_all[0] > 10.0, "including pose 0 in the mean contaminates remaining frames"

    # updateTable: row 0 unchecked -> "-", remaining rows consume packed res[0..]
    valid = [False] + [True] * (n - 1)
    cells = simulate_update_table(valid, packed)
    assert cells[0] == ("-", "-", "-")
    assert all(c != ("-", "-", "-") for c in cells[1:])
    assert len(packed) == sum(valid)
    print("table row0:", cells[0], "row1:", cells[1])
    print("OK")


if __name__ == "__main__":
    main()
