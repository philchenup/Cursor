"""Windows runtime helpers for GoTrack."""

from __future__ import annotations

import ast
import os
import sys
import unittest
from pathlib import Path
from unittest import mock

GOTRACK_ROOT = Path(__file__).resolve().parents[1]
if str(GOTRACK_ROOT) not in sys.path:
    sys.path.insert(0, str(GOTRACK_ROOT))

from utils import win_compat  # noqa: E402


def _load_slugify():
    source = (GOTRACK_ROOT / "utils" / "misc.py").read_text(encoding="utf-8")
    tree = ast.parse(source)
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name == "slugify":
            module = ast.Module(body=[node], type_ignores=[])
            namespace = {}
            exec(compile(module, "misc.py", "exec"), namespace)
            return namespace["slugify"]
    raise AssertionError("slugify not found")


class TestWinCompat(unittest.TestCase):
    def test_dataloader_workers_zero_on_windows(self):
        with mock.patch.object(win_compat, "is_windows", return_value=True):
            self.assertEqual(win_compat.dataloader_num_workers(8), 0)

    def test_dataloader_workers_passthrough_elsewhere(self):
        with mock.patch.object(win_compat, "is_windows", return_value=False):
            self.assertEqual(win_compat.dataloader_num_workers(4), 4)

    def test_mp_pool_disabled_on_windows(self):
        with mock.patch.object(win_compat, "is_windows", return_value=True):
            self.assertEqual(win_compat.mp_pool_workers(10), 0)

    def test_mp_pool_disabled_for_single_worker(self):
        with mock.patch.object(win_compat, "is_windows", return_value=False):
            self.assertEqual(win_compat.mp_pool_workers(1), 0)
            self.assertEqual(win_compat.mp_pool_workers(4), 4)

    def test_opengl_clears_egl_on_windows(self):
        env = {"PYOPENGL_PLATFORM": "egl", "DISPLAY": ":1"}
        with mock.patch.object(win_compat, "is_windows", return_value=True):
            with mock.patch.dict(os.environ, env, clear=True):
                win_compat.configure_opengl_platform()
                self.assertNotIn("PYOPENGL_PLATFORM", os.environ)
                self.assertNotIn("DISPLAY", os.environ)

    def test_opengl_sets_egl_on_linux(self):
        with mock.patch.object(win_compat, "is_windows", return_value=False):
            with mock.patch.object(sys, "platform", "linux"):
                with mock.patch.dict(os.environ, {}, clear=True):
                    win_compat.configure_opengl_platform()
                    self.assertEqual(os.environ.get("PYOPENGL_PLATFORM"), "egl")

    def test_normalize_windows_paths(self):
        cfg = {
            "save_dir": r"D:\gotrack_project\results\run",
            "user": {"root_dir": r"D:\gotrack_project"},
            "machine": {"root_dir": r"D:\gotrack_project"},
        }
        with mock.patch.object(win_compat, "is_windows", return_value=True):
            win_compat.normalize_windows_paths(cfg)
        save_dir = cfg["save_dir"].replace("\\", "/")
        root = cfg["user"]["root_dir"].replace("\\", "/")
        self.assertIn("gotrack_project/results/run", save_dir)
        self.assertTrue(root.endswith("gotrack_project"))

    def test_hydra_hint_uses_set_on_windows(self):
        with mock.patch.object(win_compat, "is_windows", return_value=True):
            hint = win_compat.hydra_full_error_hint()
        self.assertIn("set HYDRA_FULL_ERROR=1", hint)
        self.assertNotIn("export HYDRA_FULL_ERROR", hint)

    def test_renderer_does_not_force_egl(self):
        text = (GOTRACK_ROOT / "utils" / "renderer.py").read_text(encoding="utf-8")
        self.assertIn("configure_opengl_platform()", text)
        self.assertNotIn('os.environ["PYOPENGL_PLATFORM"] = "egl"', text)

    def test_template_util_uses_mp_pool_workers(self):
        text = (GOTRACK_ROOT / "utils" / "template_util.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("mp_pool_workers", text)
        self.assertIn("Windows spawn", text)

    def test_inference_scripts_force_windows_dataloader_workers(self):
        gotrack = (
            GOTRACK_ROOT / "scripts" / "inference_gotrack.py"
        ).read_text(encoding="utf-8")
        pose = (
            GOTRACK_ROOT / "scripts" / "inference_pose_estimation.py"
        ).read_text(encoding="utf-8")
        self.assertIn("dataloader_num_workers", gotrack)
        self.assertIn("normalize_windows_paths", gotrack)
        self.assertIn("dataloader_num_workers", pose)
        self.assertIn("normalize_windows_paths", pose)

    def test_slugify_windows_path(self):
        slugify = _load_slugify()
        self.assertEqual(
            slugify(r"D:\gotrack_project\results"),
            "D:-gotrack_project-results",
        )


if __name__ == "__main__":
    unittest.main()
