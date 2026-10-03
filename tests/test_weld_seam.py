import unittest
from pathlib import Path

import numpy as np

from weld_seam.detect import (
    cloud_from_screenshot,
    find_groove_seams,
    find_seams_in_image,
    fit_two_plate_seam,
    tjoint_seams_3d,
)

ROOT = Path(__file__).resolve().parents[1]
IMAGE = ROOT / "weld_seam" / "examples" / "t_plates.png"


def grid(x0, x1, y0, y1, step=1.0):
    xs = np.arange(x0, x1 + 1e-9, step)
    ys = np.arange(y0, y1 + 1e-9, step)
    xx, yy = np.meshgrid(xs, ys)
    return np.stack([xx.ravel(), yy.ravel()], axis=1)


def rotate(points, degrees, origin=(0.0, 0.0)):
    theta = np.radians(degrees)
    rotation = np.array(
        [[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]],
        dtype=np.float64,
    )
    origin = np.asarray(origin, dtype=np.float64)
    return (points - origin) @ rotation.T + origin


class TwoPlateSeamTest(unittest.TestCase):
    def test_axis_aligned_gap_center_and_toes(self):
        plate_a = grid(0, 40, 0, 200)
        plate_b = grid(55, 95, 0, 200)
        seam = fit_two_plate_seam(plate_a, plate_b)
        self.assertIsNotNone(seam)
        self.assertAlmostEqual(seam.centerline[:, 0].mean(), 47.5, delta=0.6)
        self.assertAlmostEqual(seam.toe_a[:, 0].mean(), 40.0, delta=0.8)
        self.assertAlmostEqual(seam.toe_b[:, 0].mean(), 55.0, delta=0.8)
        self.assertAlmostEqual(seam.length, 196.0, delta=3.0)
        self.assertLess(seam.gap_mad, 0.2)
        self.assertLess(seam.parallel_angle_deg, 1.0)

    def test_rotation_noise_hole_and_outliers_keep_the_centerline(self):
        plate_a = grid(0, 40, 0, 200)
        hole = (plate_a[:, 0] - 15) ** 2 + (plate_a[:, 1] - 100) ** 2 < 12**2
        plate_a = plate_a[~hole]
        plate_b = grid(55, 95, 0, 200)
        rng = np.random.default_rng(7)
        plate_a = plate_a + rng.normal(0, 0.25, plate_a.shape)
        plate_b = plate_b + rng.normal(0, 0.25, plate_b.shape)
        outliers = np.column_stack(
            [rng.uniform(43, 52, 40), rng.uniform(0, 200, 40)]
        )
        plate_a = np.vstack([plate_a, outliers])
        plate_a = rotate(plate_a, 37)
        plate_b = rotate(plate_b, 37)

        seam = fit_two_plate_seam(plate_a, plate_b)
        self.assertIsNotNone(seam)
        midpoint = rotate(seam.centerline.mean(axis=0), -37)
        self.assertAlmostEqual(midpoint[0], 47.5, delta=1.0)
        expected = rotate(np.array([0.0, 1.0]), 37)
        self.assertGreater(abs(np.dot(seam.direction, expected)), 0.998)
        self.assertLess(seam.gap_mad, 1.5)

    def test_end_to_end_and_distant_parallel_plates_are_rejected(self):
        left = grid(0, 40, 0, 100)
        right = grid(0, 40, 140, 240)
        self.assertIsNone(fit_two_plate_seam(left, right))

        near = grid(0, 40, 0, 200)
        far = grid(200, 240, 0, 200)
        self.assertIsNone(fit_two_plate_seam(near, far))

        angled = rotate(grid(55, 95, 0, 200), 40)
        self.assertIsNone(fit_two_plate_seam(near, angled))


class ThreeDimensionalTJointTest(unittest.TestCase):
    def test_unbalanced_web_faces_do_not_shift_the_centerline(self):
        flange = grid(-80, 80, -80, 80, step=1.0)
        flange = np.column_stack([flange, np.zeros(len(flange))])
        y = np.arange(-40, 41, 1.0)
        z = np.arange(2.0, 26.0, 1.0)
        yy, zz = np.meshgrid(y, z)
        face_neg = np.column_stack([np.full(yy.size, -2.0), yy.ravel(), zz.ravel()])
        face_pos = np.column_stack([np.full(yy.size, 2.0), yy.ravel(), zz.ravel()])
        face_pos = np.vstack([face_pos] * 5)
        rng = np.random.default_rng(3)
        points = np.vstack([flange, face_neg, face_pos])
        points = points + rng.normal(0, 0.05, points.shape)
        outliers = np.column_stack(
            [
                np.full(30, 30.0),
                rng.uniform(-2, 2, 30),
                np.full(30, 10.0),
            ]
        )
        points = np.vstack([points, outliers])

        seam = tjoint_seams_3d(points, bin_size=2.0)
        self.assertIsNotNone(seam)
        self.assertAlmostEqual(seam.centerline[:, 0].mean(), 0.0, delta=0.45)
        self.assertLess(np.max(np.abs(seam.centerline[:, 2])), 0.4)
        sides = sorted(
            [seam.toe_a[:, 0].mean(), seam.toe_b[:, 0].mean()]
        )
        self.assertAlmostEqual(sides[0], -2.0, delta=0.7)
        self.assertAlmostEqual(sides[1], 2.0, delta=0.7)
        self.assertGreater(abs(np.dot(seam.direction, np.array([0.0, 1.0, 0.0]))), 0.99)
        self.assertGreater(seam.length, 70.0)


class PointCloudGapTest(unittest.TestCase):
    def test_round_hole_and_scratch_are_not_welds(self):
        xs = np.arange(0, 220)
        ys = np.arange(0, 320)
        xx, yy = np.meshgrid(xs, ys)
        points = np.stack([xx.ravel(), yy.ravel()], axis=1).astype(np.float64)
        groove = (
            (points[:, 0] >= 100)
            & (points[:, 0] <= 130)
            & (points[:, 1] >= 30)
            & (points[:, 1] <= 290)
        )
        hole = (points[:, 0] - 40) ** 2 + (points[:, 1] - 50) ** 2 <= 8**2
        scratch = (
            (points[:, 0] >= 180)
            & (points[:, 0] <= 181)
            & (points[:, 1] >= 100)
            & (points[:, 1] <= 180)
        )
        seams = find_groove_seams(points[~(groove | hole | scratch)])
        self.assertEqual(len(seams), 1)
        seam = seams[0]
        self.assertAlmostEqual(float(seam.centerline[:, 0].mean()), 115.0, delta=2.0)
        self.assertGreater(seam.width, 24.0)
        self.assertLess(seam.width, 34.0)
        self.assertGreater(seam.length, 220.0)
        self.assertLess(seam.gap_mad, 1.0)


class ImageSeamTest(unittest.TestCase):
    def test_two_t_plates_on_the_scan(self):
        import cv2

        image = cv2.imread(str(IMAGE), cv2.IMREAD_COLOR)
        self.assertIsNotNone(image)
        rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        cloud = cloud_from_screenshot(rgb)
        self.assertEqual(cloud.shape[1], 3)
        self.assertGreater(len(cloud), 1_000_000)
        self.assertTrue(np.all(cloud[:, 2] == 0))
        seams = find_seams_in_image(rgb)
        self.assertEqual(len(seams), 2)

        tilted, upright = seams
        self.assertGreater(abs(np.dot(upright.direction, np.array([0.0, 1.0]))), 0.995)
        self.assertAlmostEqual(upright.centerline[:, 0].mean(), 1026.5, delta=4.0)
        self.assertGreater(upright.width, 44.0)
        self.assertLess(upright.width, 56.0)
        self.assertGreater(upright.length, 440.0)
        self.assertLess(upright.gap_mad, 2.0)
        self.assertLess(upright.parallel_angle_deg, 2.0)

        self.assertAlmostEqual(tilted.direction_deg, 49.0, delta=4.0)
        self.assertGreater(tilted.width, 68.0)
        self.assertLess(tilted.width, 82.0)
        self.assertGreater(tilted.length, 450.0)
        self.assertLess(tilted.gap_mad, 2.0)
        self.assertLess(tilted.parallel_angle_deg, 2.0)

        for seam in seams:
            mid = seam.centerline.mean(axis=0)
            toe_mid = 0.5 * (seam.toe_a.mean(axis=0) + seam.toe_b.mean(axis=0))
            self.assertLess(np.linalg.norm(mid - toe_mid), 0.05)
            self.assertGreater(np.dot(seam.toe_b[0] - seam.toe_a[0], seam.normal), 0)


if __name__ == "__main__":
    unittest.main()
