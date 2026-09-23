#!/usr/bin/env python3
"""Regression: a red ridge strip sitting on the blue face must flip back to blue."""

from __future__ import annotations

import numpy as np


def pick_plane(p, nrm, planes, dist_th=1.2, normal_th=0.88, band=1.0):
    best = second = -1
    best_align = second_align = -1.0
    best_d = second_d = 1e9
    for k, (n, b) in enumerate(planes):
        d = abs(float(n @ p + b))
        if d > dist_th:
            continue
        align = abs(float(nrm @ n))
        if align < normal_th:
            continue
        better = align > best_align + 0.04 or (abs(align - best_align) <= 0.04 and d < best_d)
        if better:
            second, second_align, second_d = best, best_align, best_d
            best, best_align, best_d = k, align, d
        elif align > second_align or (abs(align - second_align) <= 0.04 and d < second_d):
            second, second_align, second_d = k, align, d
    if best >= 0 and second >= 0 and abs(best_align - second_align) < 0.08 and best_d < band and second_d < band:
        return -1
    return best


def nearest_plane(pts, nrms, planes):
    return np.array([pick_plane(p, n, planes) for p, n in zip(pts, nrms)], dtype=int)


def knn_smooth(pts, labels, k=16):
    from collections import Counter

    next_lab = labels.copy()
    for i, p in enumerate(pts):
        dist = np.linalg.norm(pts - p, axis=1)
        nn = np.argsort(dist)[:k]
        votes = [int(labels[j]) for j in nn if labels[j] >= 0]
        if len(votes) < 3:
            continue
        maj, cnt = Counter(votes).most_common(1)[0]
        if cnt * 2 > len(votes):
            next_lab[i] = maj
    return next_lab


def make_scene():
    rng = np.random.default_rng(0)
    # red: z=0, x in [0,80], y in [0,60]
    red = np.column_stack(
        [rng.uniform(0, 80, 4000), rng.uniform(0, 60, 4000), rng.normal(0, 0.15, 4000)]
    )
    # green: y=0, x in [0,80], z in [-50,0]
    green = np.column_stack(
        [rng.uniform(0, 80, 3500), rng.normal(0, 0.15, 3500), rng.uniform(-50, 0, 3500)]
    )
    # blue: x=80, y in [0,60], z in [-50,0]
    blue = np.column_stack(
        [np.full(3500, 80.0) + rng.normal(0, 0.15, 3500), rng.uniform(0, 60, 3500), rng.uniform(-50, 0, 3500)]
    )
    # leaked strip ON the blue face, still within 0.6 mm of the red plane.
    # Sequential RANSAC (distance only) paints it red.
    leak = np.column_stack(
        [
            np.full(240, 80.0) + rng.normal(0, 0.10, 240),
            rng.uniform(5, 55, 240),
            rng.uniform(-0.8, -0.2, 240),
        ]
    )
    n_red = np.tile(np.array([0.0, 0.0, 1.0]), (len(red), 1))
    n_green = np.tile(np.array([0.0, 1.0, 0.0]), (len(green), 1))
    n_blue = np.tile(np.array([1.0, 0.0, 0.0]), (len(blue), 1))
    n_leak = np.tile(np.array([1.0, 0.0, 0.0]), (len(leak), 1))
    planes = [
        (np.array([0.0, 0.0, 1.0]), 0.0),  # red
        (np.array([0.0, 1.0, 0.0]), 0.0),  # green
        (np.array([1.0, 0.0, 0.0]), -80.0),  # blue
    ]
    return red, green, blue, leak, n_red, n_green, n_blue, n_leak, planes


def main() -> int:
    red, green, blue, leak, n_red, n_green, n_blue, n_leak, planes = make_scene()
    all_pts = np.vstack([red, green, blue, leak])
    all_n = np.vstack([n_red, n_green, n_blue, n_leak])
    labels = nearest_plane(all_pts, all_n, planes)
    labels = knn_smooth(all_pts, labels)

    n_red, n_green, n_blue = len(red), len(green), len(blue)
    leak_labels = labels[n_red + n_green + n_blue :]
    blue_share = float(np.mean(leak_labels == 2))
    red_share = float(np.mean(leak_labels == 0))
    print(f"leak -> blue {blue_share:.2f}  red {red_share:.2f}  rest {float(np.mean(leak_labels < 0)):.2f}")
    if red_share > 0.15:
        raise SystemExit("leaked strip is still mostly red")
    if blue_share + float(np.mean(leak_labels < 0)) < 0.85:
        raise SystemExit("leaked strip was not reclaimed by blue/rest")
    print("ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
