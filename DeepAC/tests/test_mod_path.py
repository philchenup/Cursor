"""Regression tests for Windows module-path resolution in DeepAC get_class.

The original code did ``file[:-3].replace('/', '.')`` and then
``whole_path.find(base_path)``. On Windows the path uses backslashes, so
``find`` returns -1 and the sliced result is the last character of the
filename. For ``deep_ac.py`` that character is ``'c'``, which matches the
user-reported ``ModuleNotFoundError: No module named 'c'``.
"""
import ast
import os
import unittest
from pathlib import Path


UTILS_PY = Path(__file__).resolve().parents[1] / 'src_open' / 'utils' / 'utils.py'

# Path from the user traceback (Windows DeepAC-main checkout).
USER_WINDOWS_FILE = r'E:\algorithm\screwTrack\DeepAC-main\src_open\models\deep_ac.py'
BASE_PATH = 'src_open.models'
EXPECTED_MOD_PATH = 'src_open.models.deep_ac'


def load_file_path_to_mod_path():
    """Load file_path_to_mod_path from utils.py without importing torch."""
    source = UTILS_PY.read_text(encoding='utf-8')
    tree = ast.parse(source, filename=str(UTILS_PY))
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name == 'file_path_to_mod_path':
            module = ast.Module(body=[node], type_ignores=[])
            namespace = {'os': os}
            exec(compile(module, str(UTILS_PY), 'exec'), namespace)
            return namespace['file_path_to_mod_path']
    raise AssertionError('file_path_to_mod_path not found in {}'.format(UTILS_PY))


def old_buggy_mod_path(file_path, base_path):
    """Reproduce the original Windows-broken conversion."""
    whole_path = file_path[:-3].replace('/', '.')
    p = whole_path.find(base_path)
    return whole_path[p:]


class TestFilePathToModPath(unittest.TestCase):
    def setUp(self):
        # Keep the helper on the instance so unittest does not bind it as a method.
        self.convert = load_file_path_to_mod_path()

    def test_old_logic_reproduces_no_module_named_c(self):
        self.assertEqual(old_buggy_mod_path(USER_WINDOWS_FILE, BASE_PATH), 'c')

    def test_windows_user_path_resolves_to_deep_ac(self):
        self.assertEqual(
            self.convert(USER_WINDOWS_FILE, BASE_PATH),
            EXPECTED_MOD_PATH,
        )

    def test_posix_path_still_resolves(self):
        posix_file = '/home/user/DeepAC/src_open/models/deep_ac.py'
        self.assertEqual(self.convert(posix_file, BASE_PATH), EXPECTED_MOD_PATH)

    def test_mixed_separators(self):
        mixed = r'E:\algorithm\screwTrack\DeepAC-main/src_open/models/deep_ac.py'
        self.assertEqual(self.convert(mixed, BASE_PATH), EXPECTED_MOD_PATH)

    def test_nested_dataset_module(self):
        windows_file = r'D:\code\DeepAC\src_open\dataset\BOP.py'
        self.assertEqual(
            self.convert(windows_file, 'src_open.dataset'),
            'src_open.dataset.BOP',
        )

    def test_missing_package_raises(self):
        with self.assertRaises(ModuleNotFoundError):
            self.convert(r'C:\tmp\unrelated\foo.py', BASE_PATH)


if __name__ == '__main__':
    unittest.main()
