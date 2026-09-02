"""GoTrack Hydra ``machine.trainer`` / pytorch_lightning.Trainer fix.

Reproduces the upstream error Hydra prints when it cannot import
``pytorch_lightning.Trainer``, and checks that the shim locates Trainer
from either ``pytorch_lightning`` or ``lightning.pytorch``.
"""

from __future__ import annotations

import importlib
import sys
import types
import unittest
from pathlib import Path
from unittest import mock

GOTRACK_ROOT = Path(__file__).resolve().parents[1]
if str(GOTRACK_ROOT) not in sys.path:
    sys.path.insert(0, str(GOTRACK_ROOT))

from utils import pl_compat  # noqa: E402


class FakeTrainer:
    def __init__(self, **kwargs):
        self.kwargs = kwargs


class FakeLogger:
    def __init__(self, **kwargs):
        self.kwargs = kwargs


class FakeCallback:
    def __init__(self, **kwargs):
        self.kwargs = kwargs


class FakeLightningModule:
    pass


def _install_fake_lightning(module_name: str) -> types.ModuleType:
    """Register a minimal lightning package in sys.modules."""
    root = types.ModuleType(module_name)
    root.Trainer = FakeTrainer
    root.LightningModule = FakeLightningModule

    loggers = types.ModuleType(module_name + ".loggers")
    loggers.TensorBoardLogger = FakeLogger
    tensorboard = types.ModuleType(module_name + ".loggers.tensorboard")
    tensorboard.TensorBoardLogger = FakeLogger
    loggers.tensorboard = tensorboard

    callbacks = types.ModuleType(module_name + ".callbacks")
    callbacks.LearningRateMonitor = FakeCallback
    callbacks.ModelCheckpoint = FakeCallback
    lr_monitor = types.ModuleType(module_name + ".callbacks.lr_monitor")
    lr_monitor.LearningRateMonitor = FakeCallback
    model_checkpoint = types.ModuleType(module_name + ".callbacks.model_checkpoint")
    model_checkpoint.ModelCheckpoint = FakeCallback
    callbacks.lr_monitor = lr_monitor
    callbacks.model_checkpoint = model_checkpoint

    root.loggers = loggers
    root.callbacks = callbacks

    sys.modules[module_name] = root
    sys.modules[loggers.__name__] = loggers
    sys.modules[tensorboard.__name__] = tensorboard
    sys.modules[callbacks.__name__] = callbacks
    sys.modules[lr_monitor.__name__] = lr_monitor
    sys.modules[model_checkpoint.__name__] = model_checkpoint
    return root


def _purge_lightning():
    doomed = [
        name
        for name in list(sys.modules)
        if name == "pytorch_lightning"
        or name.startswith("pytorch_lightning.")
        or name == "lightning"
        or name.startswith("lightning.")
    ]
    for name in doomed:
        sys.modules.pop(name, None)


class TestRewriteTargets(unittest.TestCase):
    def test_rewrites_machine_trainer_target(self):
        cfg = {
            "_target_": "pytorch_lightning.Trainer",
            "logger": {
                "_target_": "pytorch_lightning.loggers.TensorBoardLogger",
                "name": "gotrack",
            },
            "callbacks": [
                {"_target_": "pytorch_lightning.callbacks.ModelCheckpoint"},
                {"_target_": "pytorch_lightning.callbacks.LearningRateMonitor"},
            ],
        }
        pl_compat.rewrite_lightning_targets(cfg)
        self.assertEqual(cfg["_target_"], "utils.pl_compat.Trainer")
        self.assertEqual(
            cfg["logger"]["_target_"], "utils.pl_compat.TensorBoardLogger"
        )
        self.assertEqual(
            cfg["callbacks"][0]["_target_"], "utils.pl_compat.ModelCheckpoint"
        )
        self.assertEqual(
            cfg["callbacks"][1]["_target_"], "utils.pl_compat.LearningRateMonitor"
        )

    def test_patched_yaml_uses_shim_target(self):
        text = (
            GOTRACK_ROOT / "configs" / "machine" / "trainer" / "local.yaml"
        ).read_text(encoding="utf-8")
        self.assertIn("_target_: utils.pl_compat.Trainer", text)
        self.assertNotIn("pytorch_lightning.Trainer", text)


class TestAdaptTrainerKwargs(unittest.TestCase):
    def test_cpu_drops_gpu_and_ddp(self):
        with mock.patch.object(pl_compat, "_cuda_available", return_value=False):
            adapted = pl_compat.adapt_trainer_kwargs(
                {
                    "accelerator": "gpu",
                    "devices": [0],
                    "strategy": "ddp",
                    "max_epochs": 1000,
                }
            )
        self.assertEqual(adapted["accelerator"], "cpu")
        self.assertEqual(adapted["devices"], 1)
        self.assertNotIn("strategy", adapted)
        self.assertEqual(adapted["max_epochs"], 1000)

    def test_multi_gpu_keeps_ddp_when_cuda_exists(self):
        with mock.patch.object(pl_compat, "_cuda_available", return_value=True):
            adapted = pl_compat.adapt_trainer_kwargs(
                {
                    "accelerator": "gpu",
                    "devices": [0, 1],
                    "strategy": "ddp",
                }
            )
        self.assertEqual(adapted["accelerator"], "gpu")
        self.assertEqual(adapted["devices"], [0, 1])
        self.assertEqual(adapted["strategy"], "ddp")

    def test_single_gpu_drops_ddp(self):
        with mock.patch.object(pl_compat, "_cuda_available", return_value=True):
            adapted = pl_compat.adapt_trainer_kwargs(
                {
                    "accelerator": "gpu",
                    "devices": [0],
                    "strategy": "ddp",
                }
            )
        self.assertNotIn("strategy", adapted)


class TestImportResolution(unittest.TestCase):
    def tearDown(self):
        _purge_lightning()

    def test_prefers_pytorch_lightning(self):
        _purge_lightning()
        _install_fake_lightning("pytorch_lightning")
        module = pl_compat.import_lightning_module()
        self.assertEqual(module.__name__, "pytorch_lightning")
        self.assertIs(pl_compat.resolve_symbol("Trainer"), FakeTrainer)

    def test_falls_back_to_lightning_pytorch(self):
        _purge_lightning()
        _install_fake_lightning("lightning.pytorch")
        module = pl_compat.import_lightning_module()
        self.assertEqual(module.__name__, "lightning.pytorch")
        with mock.patch.object(pl_compat, "_cuda_available", return_value=False):
            trainer = pl_compat.Trainer(
                accelerator="gpu", devices=[0], strategy="ddp"
            )
        self.assertIsInstance(trainer, FakeTrainer)
        self.assertEqual(trainer.kwargs.get("accelerator"), "cpu")
        self.assertNotIn("strategy", trainer.kwargs)

    def test_missing_package_message_mentions_machine_trainer(self):
        _purge_lightning()
        with self.assertRaises(ImportError) as ctx:
            pl_compat.import_lightning_module()
        message = str(ctx.exception)
        self.assertIn("machine.trainer", message)
        self.assertIn("pytorch-lightning==1.8.6", message)
        self.assertIn("HYDRA_FULL_ERROR", message)


class TestHydraLocate(unittest.TestCase):
    """Reproduce the user-facing Hydra error and show the shim avoids it."""

    def tearDown(self):
        _purge_lightning()

    def test_hydra_cannot_locate_pytorch_lightning_trainer_without_package(self):
        hydra_utils = importlib.import_module("hydra.utils")
        instantiate = hydra_utils.instantiate
        _purge_lightning()
        with self.assertRaises(Exception) as ctx:
            instantiate({"_target_": "pytorch_lightning.Trainer", "max_epochs": 1})
        message = str(ctx.exception)
        self.assertIn("pytorch_lightning.Trainer", message)
        self.assertIn("Error locating target", message)

    def test_hydra_instantiates_shim_trainer(self):
        hydra_utils = importlib.import_module("hydra.utils")
        instantiate = hydra_utils.instantiate
        _purge_lightning()
        _install_fake_lightning("lightning.pytorch")
        with mock.patch.object(pl_compat, "_cuda_available", return_value=False):
            trainer = instantiate(
                {
                    "_target_": "utils.pl_compat.Trainer",
                    "accelerator": "gpu",
                    "devices": [0],
                    "strategy": "ddp",
                    "max_epochs": 1,
                }
            )
        self.assertIsInstance(trainer, FakeTrainer)
        self.assertEqual(trainer.kwargs["max_epochs"], 1)
        self.assertEqual(trainer.kwargs["accelerator"], "cpu")

    def test_instantiate_trainer_rewrites_upstream_yaml_target(self):
        _purge_lightning()
        _install_fake_lightning("pytorch_lightning")
        trainer = pl_compat.instantiate_trainer(
            {
                "_target_": "pytorch_lightning.Trainer",
                "accelerator": "cpu",
                "devices": 1,
                "max_epochs": 3,
            }
        )
        self.assertIsInstance(trainer, FakeTrainer)
        self.assertEqual(trainer.kwargs["max_epochs"], 3)


class TestPatchedInferenceScripts(unittest.TestCase):
    def test_inference_gotrack_uses_instantiate_trainer(self):
        text = (
            GOTRACK_ROOT / "scripts" / "inference_gotrack.py"
        ).read_text(encoding="utf-8")
        self.assertIn("instantiate_trainer", text)
        self.assertIn("from utils.pl_compat import instantiate_trainer", text)

    def test_inference_pose_estimation_uses_instantiate_trainer(self):
        text = (
            GOTRACK_ROOT / "scripts" / "inference_pose_estimation.py"
        ).read_text(encoding="utf-8")
        self.assertIn("from utils.pl_compat import instantiate_trainer", text)

    def test_model_base_does_not_import_pytorch_lightning_directly(self):
        text = (GOTRACK_ROOT / "model" / "base.py").read_text(encoding="utf-8")
        self.assertNotIn("import pytorch_lightning", text)
        self.assertIn("from utils.pl_compat import LightningModule", text)


if __name__ == "__main__":
    unittest.main()
