"""PyTorch Lightning import shim for GoTrack Hydra configs.

GoTrack's YAML uses ``_target_: pytorch_lightning.Trainer`` under
``machine.trainer``. Hydra then does the equivalent of
``from pytorch_lightning import Trainer``. When that import fails, Hydra
reports only:

    Error locating target 'pytorch_lightning.Trainer'
    full_key: machine.trainer

and hides the real exception unless ``HYDRA_FULL_ERROR=1``.

That import fails when:

* ``pytorch-lightning`` is not installed
* only the Lightning 2 metapackage ``lightning`` is installed
  (``lightning.pytorch.Trainer`` exists, ``pytorch_lightning`` may not)
* ``pytorch_lightning`` itself crashes on import (version skew with
  torch / torchmetrics)

This module re-exports Trainer, loggers, and callbacks from whichever
package is available, and is the Hydra ``_target_`` used by the patched
configs. Inference scripts also rewrite leftover ``pytorch_lightning.*``
targets so an unpatched YAML still works.
"""

from __future__ import annotations

import importlib
from typing import Any, Iterable, Optional

_PL_MODULE_CANDIDATES = (
    "pytorch_lightning",
    "lightning.pytorch",
)

_TARGET_MAP = {
    "pytorch_lightning.Trainer": "utils.pl_compat.Trainer",
    "pytorch_lightning.trainer.Trainer": "utils.pl_compat.Trainer",
    "pytorch_lightning.trainer.trainer.Trainer": "utils.pl_compat.Trainer",
    "lightning.pytorch.Trainer": "utils.pl_compat.Trainer",
    "lightning.pytorch.trainer.Trainer": "utils.pl_compat.Trainer",
    "lightning.pytorch.trainer.trainer.Trainer": "utils.pl_compat.Trainer",
    "pytorch_lightning.loggers.TensorBoardLogger": "utils.pl_compat.TensorBoardLogger",
    "pytorch_lightning.loggers.tensorboard.TensorBoardLogger": (
        "utils.pl_compat.TensorBoardLogger"
    ),
    "lightning.pytorch.loggers.TensorBoardLogger": "utils.pl_compat.TensorBoardLogger",
    "lightning.pytorch.loggers.tensorboard.TensorBoardLogger": (
        "utils.pl_compat.TensorBoardLogger"
    ),
    "pytorch_lightning.callbacks.LearningRateMonitor": (
        "utils.pl_compat.LearningRateMonitor"
    ),
    "pytorch_lightning.callbacks.lr_monitor.LearningRateMonitor": (
        "utils.pl_compat.LearningRateMonitor"
    ),
    "lightning.pytorch.callbacks.LearningRateMonitor": (
        "utils.pl_compat.LearningRateMonitor"
    ),
    "lightning.pytorch.callbacks.lr_monitor.LearningRateMonitor": (
        "utils.pl_compat.LearningRateMonitor"
    ),
    "pytorch_lightning.callbacks.ModelCheckpoint": "utils.pl_compat.ModelCheckpoint",
    "pytorch_lightning.callbacks.model_checkpoint.ModelCheckpoint": (
        "utils.pl_compat.ModelCheckpoint"
    ),
    "lightning.pytorch.callbacks.ModelCheckpoint": "utils.pl_compat.ModelCheckpoint",
    "lightning.pytorch.callbacks.model_checkpoint.ModelCheckpoint": (
        "utils.pl_compat.ModelCheckpoint"
    ),
}

_INSTALL_HINT = (
    "GoTrack Hydra configs instantiate pytorch_lightning.Trainer "
    "(full_key: machine.trainer). Install the package from environment.yml:\n"
    "  pip install 'pytorch-lightning==1.8.6'\n"
    "Lightning 2.x also works with this shim:\n"
    "  pip install lightning\n"
    "To see Hydra's chained ImportError, run:\n"
    "  export HYDRA_FULL_ERROR=1"
)


def _format_import_errors(errors: Iterable[str]) -> str:
    body = "\n".join("  - " + item for item in errors) or "  - (no import attempted)"
    return _INSTALL_HINT + "\nTried:\n" + body


def import_lightning_module():
    """Import ``pytorch_lightning`` or ``lightning.pytorch``."""
    errors = []
    for name in _PL_MODULE_CANDIDATES:
        try:
            return importlib.import_module(name)
        except Exception as exc:  # noqa: BLE001 - surface every import failure
            errors.append("{}: {}: {}".format(name, type(exc).__name__, exc))
    raise ImportError(_format_import_errors(errors))


def resolve_symbol(dotted_suffix: str):
    """Resolve an attribute path relative to the lightning package.

    ``dotted_suffix`` examples: ``Trainer``, ``loggers.TensorBoardLogger``,
    ``callbacks.ModelCheckpoint``.
    """
    module = import_lightning_module()
    obj = module
    for part in dotted_suffix.split("."):
        try:
            obj = getattr(obj, part)
        except AttributeError:
            # Some symbols live in a submodule that is not re-exported.
            obj = importlib.import_module(module.__name__ + "." + part)
    return obj


def lightning_import_error_message() -> Optional[str]:
    """Return the install hint if Lightning cannot be imported, else None."""
    try:
        import_lightning_module()
    except ImportError as exc:
        return str(exc)
    return None


def _device_count(devices: Any) -> int:
    if devices is None:
        return 1
    if isinstance(devices, int):
        return devices
    if isinstance(devices, (list, tuple)):
        return len(devices)
    return 1


def _cuda_available() -> bool:
    try:
        import torch

        return bool(torch.cuda.is_available())
    except Exception:  # noqa: BLE001
        return False


def adapt_trainer_kwargs(kwargs: dict) -> dict:
    """Make Trainer kwargs work on CPU and single-process inference.

    Upstream ``configs/machine/trainer/local.yaml`` forces ``accelerator: gpu``
    and ``strategy: ddp``. That is unnecessary for ``trainer.test()`` on one
    device and crashes when CUDA is missing.
    """
    adapted = dict(kwargs)
    cuda = _cuda_available()
    accel = adapted.get("accelerator")
    n_devices = _device_count(adapted.get("devices"))

    if not cuda and accel in ("gpu", "cuda"):
        adapted["accelerator"] = "cpu"
        adapted["devices"] = 1
        n_devices = 1

    strategy = adapted.get("strategy")
    if strategy in (
        "ddp",
        "ddp_spawn",
        "ddp_find_unused_parameters_false",
        "ddp_find_unused_parameters_true",
    ):
        if n_devices <= 1:
            adapted.pop("strategy", None)

    return adapted


def _call_lightning(dotted_suffix: str, *args, **kwargs):
    cls = resolve_symbol(dotted_suffix)
    return cls(*args, **kwargs)


def Trainer(*args, **kwargs):
    """Hydra ``_target_`` for ``machine.trainer``."""
    return _call_lightning("Trainer", *args, **adapt_trainer_kwargs(kwargs))


def TensorBoardLogger(*args, **kwargs):
    return _call_lightning("loggers.TensorBoardLogger", *args, **kwargs)


def LearningRateMonitor(*args, **kwargs):
    return _call_lightning("callbacks.LearningRateMonitor", *args, **kwargs)


def ModelCheckpoint(*args, **kwargs):
    return _call_lightning("callbacks.ModelCheckpoint", *args, **kwargs)


def _load_lightning_module_base():
    try:
        return resolve_symbol("LightningModule")
    except ImportError:
        return object


LightningModule = _load_lightning_module_base()


def rewrite_lightning_targets(cfg: Any) -> Any:
    """Rewrite ``pytorch_lightning.*`` Hydra ``_target_`` strings in-place."""
    try:
        from omegaconf import DictConfig, ListConfig, OmegaConf, open_dict
    except ImportError:
        return _rewrite_plain(cfg)

    if isinstance(cfg, DictConfig):
        with open_dict(cfg):
            target = cfg.get("_target_")
            if isinstance(target, str) and target in _TARGET_MAP:
                cfg._target_ = _TARGET_MAP[target]
            for key in list(cfg.keys()):
                rewrite_lightning_targets(cfg[key])
        return cfg
    if isinstance(cfg, ListConfig):
        for item in cfg:
            rewrite_lightning_targets(item)
        return cfg
    if OmegaConf.is_config(cfg):
        return cfg
    return _rewrite_plain(cfg)


def _rewrite_plain(cfg: Any) -> Any:
    if isinstance(cfg, dict):
        target = cfg.get("_target_")
        if isinstance(target, str) and target in _TARGET_MAP:
            cfg["_target_"] = _TARGET_MAP[target]
        for value in cfg.values():
            _rewrite_plain(value)
    elif isinstance(cfg, list):
        for item in cfg:
            _rewrite_plain(item)
    return cfg


def instantiate_trainer(cfg_trainer: Any):
    """Instantiate ``machine.trainer`` with rewritten targets and a clear error."""
    from hydra.utils import instantiate

    try:
        from omegaconf import OmegaConf
    except ImportError:
        cfg = cfg_trainer
    else:
        if OmegaConf.is_config(cfg_trainer):
            cfg = OmegaConf.create(
                OmegaConf.to_container(cfg_trainer, resolve=True)
            )
        else:
            cfg = cfg_trainer

    rewrite_lightning_targets(cfg)
    try:
        return instantiate(cfg)
    except Exception as exc:  # noqa: BLE001
        cause = exc.__cause__ or exc.__context__ or exc
        hint = lightning_import_error_message()
        extra = "\n" + hint if hint else ""
        raise RuntimeError(
            "Failed to instantiate Hydra target for machine.trainer "
            "(originally pytorch_lightning.Trainer).{}"
            "\nUnderlying error: {}: {}".format(
                extra, type(cause).__name__, cause
            )
        ) from exc
