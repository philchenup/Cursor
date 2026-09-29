"""Mirror of include/OrderWeldTcpByOrigin.h (numpy stand-in for Eigen::Affine3f)."""

import unittest

import numpy as np


def order_weld_tcp_by_origin(tcp_weld_start, tcp_weld_end):
    if np.dot(tcp_weld_start[:3, 3], tcp_weld_start[:3, 3]) > np.dot(
        tcp_weld_end[:3, 3], tcp_weld_end[:3, 3]
    ):
        tcp_weld_start, tcp_weld_end = tcp_weld_end.copy(), tcp_weld_start.copy()
    else:
        tcp_weld_start, tcp_weld_end = tcp_weld_start.copy(), tcp_weld_end.copy()

    x = tcp_weld_end[:3, 3] - tcp_weld_start[:3, 3]
    x = x / np.linalg.norm(x)

    def set_x(tcp):
        y = np.cross(tcp[:3, 2], x)
        if np.dot(y, y) < 1e-12:
            y = tcp[:3, 1].copy()
        else:
            y = y / np.linalg.norm(y)
        tcp[:3, 0] = x
        tcp[:3, 1] = y
        z = np.cross(x, y)
        tcp[:3, 2] = z / np.linalg.norm(z)
        return tcp

    return set_x(tcp_weld_start), set_x(tcp_weld_end)


def pose(translation, x=None, y=None, z=None):
    T = np.eye(4, dtype=np.float64)
    T[:3, 3] = translation
    if x is None:
        x = np.array([1.0, 0.0, 0.0])
    if y is None:
        y = np.array([0.0, 1.0, 0.0])
    if z is None:
        z = np.array([0.0, 0.0, 1.0])
    T[:3, 0] = x
    T[:3, 1] = y
    T[:3, 2] = z
    return T


class OrderWeldTcpByOriginTest(unittest.TestCase):
    def test_swaps_when_start_is_farther_from_origin(self):
        start = pose([80.0, 10.0, 5.0], x=[-1.0, 0.0, 0.0], y=[0.0, -1.0, 0.0])
        end = pose([10.0, 0.0, 0.0], x=[-1.0, 0.0, 0.0], y=[0.0, -1.0, 0.0])
        start, end = order_weld_tcp_by_origin(start, end)

        self.assertLess(np.linalg.norm(start[:3, 3]), np.linalg.norm(end[:3, 3]))
        np.testing.assert_allclose(start[:3, 3], [10.0, 0.0, 0.0])
        np.testing.assert_allclose(end[:3, 3], [80.0, 10.0, 5.0])

        expected_x = end[:3, 3] - start[:3, 3]
        expected_x /= np.linalg.norm(expected_x)
        np.testing.assert_allclose(start[:3, 0], expected_x, atol=1e-9)
        np.testing.assert_allclose(end[:3, 0], expected_x, atol=1e-9)

    def test_keeps_order_when_start_is_already_closer(self):
        start = pose([5.0, 0.0, 0.0])
        end = pose([40.0, 30.0, 0.0])
        start, end = order_weld_tcp_by_origin(start, end)
        np.testing.assert_allclose(start[:3, 3], [5.0, 0.0, 0.0])
        np.testing.assert_allclose(end[:3, 3], [40.0, 30.0, 0.0])
        expected_x = np.array([35.0, 30.0, 0.0])
        expected_x /= np.linalg.norm(expected_x)
        np.testing.assert_allclose(start[:3, 0], expected_x, atol=1e-9)

    def test_flips_x_when_it_points_the_wrong_way(self):
        start = pose([1.0, 0.0, 0.0], x=[-1.0, 0.0, 0.0], y=[0.0, -1.0, 0.0])
        end = pose([5.0, 0.0, 0.0], x=[-1.0, 0.0, 0.0], y=[0.0, -1.0, 0.0])
        start, end = order_weld_tcp_by_origin(start, end)
        np.testing.assert_allclose(start[:3, 0], [1.0, 0.0, 0.0], atol=1e-9)
        np.testing.assert_allclose(end[:3, 0], [1.0, 0.0, 0.0], atol=1e-9)
        # right-handed
        self.assertGreater(np.dot(np.cross(start[:3, 0], start[:3, 1]), start[:3, 2]), 0.0)

    def test_right_handed_after_swap(self):
        start = pose([20.0, 8.0, 3.0])
        end = pose([2.0, 1.0, 0.5])
        start, end = order_weld_tcp_by_origin(start, end)
        for T in (start, end):
            R = T[:3, :3]
            self.assertAlmostEqual(np.linalg.det(R), 1.0, places=6)
            np.testing.assert_allclose(R @ R.T, np.eye(3), atol=1e-9)


if __name__ == "__main__":
    unittest.main()
