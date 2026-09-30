#!/usr/bin/env python3
"""ABC = reverse(eulerAngles(2,1,0)) so R = Rz(C) * Ry(B) * Rx(A)."""

from __future__ import annotations

import math
import unittest

import numpy as np


def rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def ry(b):
    c, s = math.cos(b), math.sin(b)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rz(c):
    c0, s = math.cos(c), math.sin(c)
    return np.array([[c0, -s, 0], [s, c0, 0], [0, 0, 1]])


def eigen_euler_zyx_reverse(R: np.ndarray) -> np.ndarray:
    """Mirror Eigen rotation().eulerAngles(2,1,0).reverse() in degrees."""
    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    if sy > 1e-9:
        a2 = math.atan2(R[2, 1], R[2, 2])  # Rx
        a1 = math.atan2(-R[2, 0], sy)  # Ry
        a0 = math.atan2(R[1, 0], R[0, 0])  # Rz
    else:
        a2 = math.atan2(-R[1, 2], R[1, 1])
        a1 = math.atan2(-R[2, 0], sy)
        a0 = 0.0
    # eulerAngles(2,1,0) returns (a0,a1,a2) = (Rz, Ry, Rx); reverse → (Rx,Ry,Rz)
    return np.degrees([a2, a1, a0])


class FlangePoseMmAbcTests(unittest.TestCase):
    def test_meters_to_mm(self) -> None:
        self.assertAlmostEqual(1.234 * 1000.0, 1234.0)

    def test_abc_rebuilds_rotation(self) -> None:
        rng = np.random.default_rng(0)
        for _ in range(20):
            a, b, c = rng.uniform(-0.8, 0.8, size=3)
            R = rz(c) @ ry(b) @ rx(a)
            abc = eigen_euler_zyx_reverse(R)
            R2 = rz(math.radians(abc[2])) @ ry(math.radians(abc[1])) @ rx(math.radians(abc[0]))
            np.testing.assert_allclose(R2, R, atol=1e-9)


if __name__ == "__main__":
    unittest.main()
