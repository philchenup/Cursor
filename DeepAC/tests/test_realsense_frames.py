"""RealSense color-frame helpers used by live_track (no camera required)."""
import importlib.util
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

LIVE_TRACK = Path(__file__).resolve().parents[1] / 'live_track.py'


def load_live_track():
    spec = importlib.util.spec_from_file_location('live_track', LIVE_TRACK)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class FakeColorFrame:
    def __init__(self, data, fmt, width=None, height=None):
        self._data = data
        self._fmt = fmt
        if data.ndim == 3:
            self._h, self._w = data.shape[:2]
        elif data.ndim == 2:
            self._h, self._w = data.shape
        else:
            self._h, self._w = height, width

    def get_width(self):
        return self._w

    def get_height(self):
        return self._h

    def get_data(self):
        return self._data

    def get_format(self):
        return self._fmt


class TestRealsenseFrameHelpers(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.mod = load_live_track()

    def test_no_orbbec_sdk(self):
        text = LIVE_TRACK.read_text(encoding='utf-8')
        self.assertNotIn('pyorbbecsdk', text)
        self.assertIn('pyrealsense2', text)

    def test_k_from_realsense_ppx_ppy(self):
        it = SimpleNamespace(fx=600.5, fy=601.25, ppx=320.0, ppy=240.0)
        K = self.mod.K_from_video_intrinsics(it)
        np.testing.assert_allclose(
            K, [[600.5, 0, 320.0], [0, 601.25, 240.0], [0, 0, 1]])

    def test_k_from_cx_cy_compat(self):
        it = SimpleNamespace(fx=500.0, fy=500.0, cx=100.0, cy=80.0)
        K = self.mod.K_from_video_intrinsics(it)
        np.testing.assert_allclose(
            K, [[500.0, 0, 100.0], [0, 500.0, 80.0], [0, 0, 1]])

    def test_color_frame_bgr8_passthrough(self):
        bgr = np.zeros((4, 6, 3), dtype=np.uint8)
        bgr[1, 2] = (0, 1, 2)
        out = self.mod.color_frame_to_bgr(FakeColorFrame(bgr, 'bgr8'))
        np.testing.assert_array_equal(out, bgr)
        self.assertEqual(out.shape, (4, 6, 3))

    def test_color_frame_unknown_format_raises(self):
        gray = np.zeros((2, 2), dtype=np.uint8)
        with self.assertRaises(RuntimeError) as ctx:
            self.mod.color_frame_to_bgr(FakeColorFrame(gray, 'z16'))
        self.assertIn('z16', str(ctx.exception))

    def test_format_name_from_enum_like_string(self):
        cf = FakeColorFrame(np.zeros((2, 2, 3), np.uint8), 'format.bgr8')
        self.assertEqual(self.mod._color_format_name(cf), 'bgr8')

    def test_frames_from_routes_realsense_alias(self):
        text = LIVE_TRACK.read_text(encoding='utf-8')
        self.assertIn("source in ('orbbec', 'realsense', 'rs')", text)
        self.assertIs(self.mod.frames_realsense, self.mod.frames_orbbec)


if __name__ == '__main__':
    unittest.main()
