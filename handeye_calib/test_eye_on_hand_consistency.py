#!/usr/bin/env python3
"""Verify eye-in-hand / eye-to-hand 3D consistency using every pose (no first-frame skip)."""
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


def mean_abs_error(points_per_pose):
    """points_per_pose[j][i] = point j at pose i. Return global XYZ MAE over all poses."""
    abs_err = []
    for obs in points_per_pose:
        arr = np.asarray(obs)
        mean = arr.mean(axis=0)
        abs_err.extend(np.abs(arr - mean))
    return np.mean(abs_err, axis=0)


def per_pose_mean_abs(points_per_pose):
    """Match C++ accumulatePoseConsistency: all poses enter the reference mean."""
    n_pose = len(points_per_pose[0])
    n_pts = len(points_per_pose)
    packed = []
    for i in range(n_pose):
        acc = np.zeros(3)
        for obs in points_per_pose:
            arr = np.asarray(obs)
            ref = arr.mean(axis=0)
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
    return mean_abs_error(collect_points(Ts, objp))


def eval_eye_on_hand(T_base2ends, T_cam2base, T_board2cams, objp):
    """p_end = T_base2end * T_cam2base * T_board2cam * p (OpenCV eye-to-hand poses)."""
    Ts = [Tb2e @ T_cam2base @ Tboard2cam
          for Tb2e, Tboard2cam in zip(T_base2ends, T_board2cams)]
    return mean_abs_error(collect_points(Ts, objp)), collect_points(Ts, objp)


def eval_eye_on_hand_wrong_end2base(T_end2bases, T_cam2base, T_board2cams, objp):
    """Bug: treat OpenCV base->end inputs as end->base and left-multiply."""
    Ts = [Te2b @ T_cam2base @ Tboard2cam
          for Te2b, Tboard2cam in zip(T_end2bases, T_board2cams)]
    return mean_abs_error(collect_points(Ts, objp))


def eval_eye_on_hand_double_inverse(T_base2ends, T_cam2base, T_board2cams, objp):
    """Bug: invert poses that are already base->end."""
    Ts = [invert_T(Tb2e) @ T_cam2base @ Tboard2cam
          for Tb2e, Tboard2cam in zip(T_base2ends, T_board2cams)]
    return mean_abs_error(collect_points(Ts, objp))


def eval_eye_in_hand_wrong_inverse(T_end2bases, T_cam2end, T_board2cams, objp):
    """Bug: invert the already-correct OpenCV cam2gripper T_cam2end."""
    Ts = [Te2b @ invert_T(T_cam2end) @ Tboard2cam
          for Te2b, Tboard2cam in zip(T_end2bases, T_board2cams)]
    return mean_abs_error(collect_points(Ts, objp))


def eval_eye_in_hand(T_end2bases, T_cam2end, T_board2cams, objp):
    """p_base = T_end2base * T_cam2end * T_board2cam * p (OpenCV X, no inverse)."""
    Ts = [Te2b @ T_cam2end @ Tboard2cam
          for Te2b, Tboard2cam in zip(T_end2bases, T_board2cams)]
    return mean_abs_error(collect_points(Ts, objp)), collect_points(Ts, objp)


def select_eye_in_hand_chain(T_end2bases, T_cam2end, T_board2cams, objp):
    """Mirror C++ candidate products; return (name, mae, points)."""
    X = T_cam2end
    Xi = invert_T(T_cam2end)
    Hg = T_end2bases
    Hgi = [invert_T(T) for T in T_end2bases]
    Hc = T_board2cams
    Hci = [invert_T(T) for T in T_board2cams]
    chains = [
        ("T_end2base * T_cam2end * T_board2cam",
         [a @ X @ b for a, b in zip(Hg, Hc)]),
        ("T_end2base * T_end2cam * T_board2cam",
         [a @ Xi @ b for a, b in zip(Hg, Hc)]),
        ("T_base2end * T_cam2end * T_board2cam",
         [a @ X @ b for a, b in zip(Hgi, Hc)]),
        ("T_base2end * T_end2cam * T_board2cam",
         [a @ Xi @ b for a, b in zip(Hgi, Hc)]),
        ("T_end2base * T_cam2end * T_cam2board",
         [a @ X @ b for a, b in zip(Hg, Hci)]),
        ("T_board2cam * T_cam2end * T_end2base",
         [a @ X @ b for a, b in zip(Hc, Hg)]),
    ]
    best_name, best_Ts, best_mae = None, None, None
    for name, Ts in chains:
        mae = mean_abs_error(collect_points(Ts, objp))
        if best_mae is None or np.sum(mae) < np.sum(best_mae):
            best_name, best_Ts, best_mae = name, Ts, mae
    return best_name, best_mae, collect_points(best_Ts, objp)


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
    T_base2ends = [invert_T(T) for T in T_end2bases]
    T_board2cams_on = [
        invert_T(T_cam2base) @ Te2b @ T_board2end for Te2b in T_end2bases
    ]
    # OpenCV Tsai: T_board2cam = inv(T_cam2end) * inv(T_end2base) * T_board2base
    T_board2cams_in = [
        invert_T(T_cam2end) @ invert_T(Te2b) @ T_board2base for Te2b in T_end2bases
    ]

    mae_old = eval_old_reuse(T_end2bases, T_cam2base, T_board2cams_on, objp)
    mae_on, pts_on = eval_eye_on_hand(T_base2ends, T_cam2base, T_board2cams_on, objp)
    mae_on_wrong = eval_eye_on_hand_wrong_end2base(
        T_end2bases, T_cam2base, T_board2cams_on, objp
    )
    mae_on_double = eval_eye_on_hand_double_inverse(
        T_base2ends, T_cam2base, T_board2cams_on, objp
    )
    mae_in, pts_in = eval_eye_in_hand(T_end2bases, T_cam2end, T_board2cams_in, objp)
    mae_in_wrong = eval_eye_in_hand_wrong_inverse(
        T_end2bases, T_cam2end, T_board2cams_in, objp
    )
    sel_name, sel_mae, sel_pts = select_eye_in_hand_chain(
        T_end2bases, T_cam2end, T_board2cams_in, objp
    )
    sel_invpose_name, sel_invpose_mae, _ = select_eye_in_hand_chain(
        T_base2ends, T_cam2end, T_board2cams_in, objp
    )
    sel_swap_name, sel_swap_mae, _ = select_eye_in_hand_chain(
        T_board2cams_in, T_cam2end, T_end2bases, objp
    )

    print("old reuse MAE:", mae_old)
    print("eye-on-hand MAE (base->end, all poses):", mae_on)
    print("eye-on-hand wrong end->base MAE:", mae_on_wrong)
    print("eye-on-hand double-inverse MAE:", mae_on_double)
    print("eye-in-hand MAE (all poses):", mae_in)
    print("eye-in-hand wrong inverse T_cam2end MAE:", mae_in_wrong)
    print("selected EIH chain:", sel_name, sel_mae)
    print("selected EIH chain if poses already inverted:", sel_invpose_name, sel_invpose_mae)
    print("selected EIH chain if board/robot args swapped:", sel_swap_name, sel_swap_mae)

    for Te2b, Tboard2cam in zip(T_end2bases, T_board2cams_in):
        assert np.allclose(Te2b @ T_cam2end @ Tboard2cam, T_board2base, atol=1e-9)

    assert np.all(mae_old > 10.0), "old path should show large motion-scale error"
    assert np.all(mae_on_wrong > 10.0), "passing end->base into eye-on-hand must fail"
    assert np.all(mae_on_double > 10.0), "inverting already inverted poses must fail"
    assert np.all(mae_on < 1e-9), "eye-on-hand all-pose path should be numerically zero"
    assert np.all(mae_in < 1e-9), "eye-in-hand all-pose path should be numerically zero"
    assert np.all(mae_in_wrong > 10.0), "inverting a correct T_cam2end must leak robot motion"
    assert sel_name == "T_end2base * T_cam2end * T_board2cam"
    assert np.all(sel_mae < 1e-9)
    assert sel_invpose_name == "T_base2end * T_cam2end * T_board2cam"
    assert np.all(sel_invpose_mae < 1e-9)
    assert sel_swap_name == "T_board2cam * T_cam2end * T_end2base"
    assert np.all(sel_swap_mae < 1e-9)

    packed_on = per_pose_mean_abs(pts_on)
    packed_in = per_pose_mean_abs(sel_pts)
    assert len(packed_on) == n, "eye-on-hand must report every pose, including the first"
    assert len(packed_in) == n, "eye-in-hand must report every pose, including the first"
    for e in packed_on + packed_in:
        assert np.all(e < 1e-9)

    # A first-pose outlier now contributes to the mean (no skip). Pose 0 is still reported.
    pts_outlier = [list(obs) for obs in pts_on]
    for j in range(len(pts_outlier)):
        pts_outlier[j][0] = np.asarray(pts_outlier[j][0]) + np.array([800.0, 50.0, 0.0])

    packed_out = per_pose_mean_abs(pts_outlier)
    assert len(packed_out) == n
    assert packed_out[0][0] > 10.0, "first pose must be included and show the injected outlier"
    # Remaining poses pick up a small leaked bias from the shared mean; they stay finite.
    for e in packed_out[1:]:
        assert np.all(np.isfinite(e))
        assert e[0] > 0.0

    # updateTable: every valid row consumes the matching packed result, including row 0.
    valid = [True] * n
    cells = simulate_update_table(valid, packed_on)
    assert all(c != ("-", "-", "-") for c in cells)
    assert cells[0] == tuple(f"{v:.4f}" for v in packed_on[0])
    assert len(packed_on) == sum(valid)

    # Unchecked middle row still aligns: later packed values map to later valid rows.
    valid_gap = [True, True, False, True] + [True] * (n - 4)
    packed_gap = packed_on
    cells_gap = simulate_update_table(valid_gap, packed_gap)
    assert cells_gap[2] == ("-", "-", "-")
    assert cells_gap[0] == tuple(f"{v:.4f}" for v in packed_gap[0])
    assert cells_gap[1] == tuple(f"{v:.4f}" for v in packed_gap[1])
    assert cells_gap[3] == tuple(f"{v:.4f}" for v in packed_gap[2])

    print("table row0:", cells[0], "row1:", cells[1])
    print("OK")


if __name__ == "__main__":
    main()
