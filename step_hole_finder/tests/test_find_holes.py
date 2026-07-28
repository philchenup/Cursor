"""Regression tests for STEP 8 mm hole detection."""

from __future__ import annotations

import unittest
from pathlib import Path

from step_hole_finder.find_holes import find_cylindrical_holes, load_step
from step_hole_finder.generate_sample import build_sample_plate

import cadquery as cq


SAMPLES = Path(__file__).resolve().parents[1] / "samples"
SAMPLE_STEP = SAMPLES / "plate_with_holes.step"


def _ensure_sample() -> Path:
    SAMPLES.mkdir(parents=True, exist_ok=True)
    if not SAMPLE_STEP.exists():
        cq.exporters.export(build_sample_plate(), str(SAMPLE_STEP))
    return SAMPLE_STEP


class FindHolesTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.step_path = _ensure_sample()
        cls.shape = load_step(cls.step_path)

    def test_finds_exactly_two_8mm_holes(self) -> None:
        holes = find_cylindrical_holes(self.shape, target_diameter_mm=8.0, diameter_tolerance_mm=0.05)
        self.assertEqual(len(holes), 2)
        for hole in holes:
            self.assertAlmostEqual(hole.diameter_mm, 8.0, delta=0.05)

    def test_lists_all_holes_when_diameter_unfiltered(self) -> None:
        holes = find_cylindrical_holes(self.shape, target_diameter_mm=None)
        diameters = sorted(round(h.diameter_mm, 3) for h in holes)
        # 6, 8, 8, 10 — boss should be excluded
        self.assertEqual(diameters, [6.0, 8.0, 8.0, 10.0])


if __name__ == "__main__":
    unittest.main()
