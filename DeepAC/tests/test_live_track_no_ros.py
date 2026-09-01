"""ROS-free live_track output helpers and CLI."""
import importlib.util
import io
import os
import tempfile
import unittest
from contextlib import redirect_stdout
from pathlib import Path

import numpy as np

LIVE_TRACK = Path(__file__).resolve().parents[1] / 'live_track.py'
FORBIDDEN = (
    'rclpy', 'sensor_msgs', 'tf2_ros', 'geometry_msgs',
    'PoseStamped', 'TransformBroadcaster', 'CameraInfo',
    'CompressedImage', 'qos_profile_sensor_data',
)


def load_live_track():
    spec = importlib.util.spec_from_file_location('live_track', LIVE_TRACK)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class TestNoRosDependency(unittest.TestCase):
    def test_source_has_no_ros_imports(self):
        text = LIVE_TRACK.read_text(encoding='utf-8')
        for token in FORBIDDEN:
            self.assertNotIn(token, text, 'found leftover ROS token: ' + token)
        self.assertNotIn('import rclpy', text)
        self.assertNotIn('from rclpy', text)

    def test_module_imports_with_numpy_only(self):
        mod = load_live_track()
        self.assertTrue(hasattr(mod, 'PoseOutput'))
        self.assertTrue(hasattr(mod, 'format_pose_line'))
        self.assertTrue(hasattr(mod, 'quat_from_R'))

    def test_parser_has_output_not_ros_flags(self):
        mod = load_live_track()
        p = mod.build_parser()
        names = {a.dest for a in p._actions}
        self.assertIn('output', names)
        self.assertIn('no_print', names)
        self.assertNotIn('publish_pose', names)
        self.assertNotIn('pose_topic', names)
        self.assertNotIn('publish_tf', names)
        self.assertNotIn('tracking_topic', names)
        self.assertNotIn('camera_frame', names)

    def test_ros2_source_exits(self):
        mod = load_live_track()
        with self.assertRaises(SystemExit) as ctx:
            mod.frames_from('ros2:/camera/color/image_raw')
        self.assertIn('ROS', str(ctx.exception))


class TestQuatAndPoseLine(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.mod = load_live_track()

    def test_identity_quat(self):
        x, y, z, w = self.mod.quat_from_R(np.eye(3))
        self.assertAlmostEqual(x, 0.0, places=6)
        self.assertAlmostEqual(y, 0.0, places=6)
        self.assertAlmostEqual(z, 0.0, places=6)
        self.assertAlmostEqual(w, 1.0, places=6)

    def test_rotz_90_quat(self):
        R = self.mod.rot_axis([0, 0, 1], 90)
        x, y, z, w = self.mod.quat_from_R(R)
        self.assertAlmostEqual(x, 0.0, places=5)
        self.assertAlmostEqual(y, 0.0, places=5)
        self.assertAlmostEqual(abs(z), np.sin(np.pi / 4), places=5)
        self.assertAlmostEqual(abs(w), np.cos(np.pi / 4), places=5)
        self.assertGreater(w * z, 0.0)  # same sign for +90 about Z

    def test_format_track_line(self):
        R = np.eye(3)
        t = np.array([0.01, -0.02, 0.25])
        met = {'conf': 0.8, 'cs': 0.7, 'dt': 1.2, 'dr': 0.5}
        line = self.mod.format_pose_line(
            12, 'TRACK', True, R, t, met, stamp_sec=123.456)
        parts = line.split()
        self.assertEqual(parts[1], '12')
        self.assertEqual(parts[2], 'TRACK')
        self.assertEqual(parts[3], '1')
        self.assertAlmostEqual(float(parts[8]), 0.01, places=6)
        self.assertAlmostEqual(float(parts[9]), -0.02, places=6)
        self.assertAlmostEqual(float(parts[10]), 0.25, places=6)
        self.assertAlmostEqual(float(parts[14]), 1.0, places=5)  # qw

    def test_format_init_nans(self):
        line = self.mod.format_pose_line(
            0, 'INIT', False, None, None, {'conf': 0.1}, stamp_sec=1.0)
        parts = line.split()
        self.assertEqual(parts[2], 'INIT')
        self.assertEqual(parts[3], '0')
        self.assertTrue(np.isnan(float(parts[8])))

    def test_pose_output_writes_file_skips_init_by_default(self):
        mod = self.mod

        class A:
            no_print = True
            print_init = False
            output = None

        with tempfile.TemporaryDirectory() as td:
            a = A()
            a.output = os.path.join(td, 'pose.txt')
            pub = mod.PoseOutput(a)
            pub.send(None, None, 0, 'INIT', False, {'conf': 0.1, 'cs': 0.1,
                                                    'dt': 0, 'dr': 0})
            R = np.eye(3)
            t = np.array([0.0, 0.0, 0.2])
            pub.send(R, t, 1, 'TRACK', True, {'conf': 0.9, 'cs': 0.8,
                                              'dt': 0.1, 'dr': 0.2})
            pub.close()
            lines = Path(a.output).read_text(encoding='utf-8').strip().splitlines()
            self.assertTrue(lines[0].startswith('#'))
            body = [ln for ln in lines[1:] if ln.strip()]
            self.assertEqual(len(body), 1)
            self.assertIn('TRACK', body[0])

    def test_pose_output_stdout_track(self):
        mod = self.mod

        class A:
            no_print = False
            print_init = False
            output = ''

        buf = io.StringIO()
        with redirect_stdout(buf):
            pub = mod.PoseOutput(A())
            pub.send(np.eye(3), np.array([0.0, 0.0, 0.25]), 3, 'TRACK', True,
                     {'conf': 0.5, 'cs': 0.5, 'dt': 0.0, 'dr': 0.0})
            pub.close()
        out = buf.getvalue()
        self.assertIn('TRACK', out)
        self.assertIn(' 3 ', out)


class TestParserHelp(unittest.TestCase):
    def test_help_mentions_stdout_not_ros2_source(self):
        text = LIVE_TRACK.read_text(encoding='utf-8')
        self.assertIn('--output', text)
        self.assertIn('cam:0', text)
        self.assertIn('realsense', text)
        self.assertNotIn('--publish-pose', text)
        self.assertNotIn('ros2:/camera', text)
        self.assertNotIn('pyorbbecsdk', text)


if __name__ == '__main__':
    unittest.main()
