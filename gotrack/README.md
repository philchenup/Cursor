# GoTrack: Hydra cannot locate `pytorch_lightning.Trainer`

Drop-in for [facebookresearch/gotrack](https://github.com/facebookresearch/gotrack).

## Error

```text
Error executing job with overrides: []
Error locating target 'pytorch_lightning.Trainer', set env var HYDRA_FULL_ERROR=1 to see chained exception.
full_key: machine.trainer
```

Hydra is instantiating `configs/machine/trainer/local.yaml`:

```yaml
_target_: pytorch_lightning.Trainer
```

That is `from pytorch_lightning import Trainer`. The real exception is hidden unless you run:

```bash
export HYDRA_FULL_ERROR=1
python -m scripts.inference_gotrack
```

Typical causes:

1. `pytorch-lightning` is not installed
2. Only Lightning 2 (`pip install lightning`) is installed, so the import path is `lightning.pytorch.Trainer`
3. `pytorch_lightning` fails to import because of a torch / torchmetrics mismatch

## Quickest environment fix

From GoTrack `environment.yml`:

```bash
pip install 'pytorch-lightning==1.8.6' hydra-core==1.3.2
python -c "from pytorch_lightning import Trainer; print(Trainer)"
```

If you already use Lightning 2.x, keep it and apply the files below instead of pinning 1.8.6.

## Apply this drop-in

From a GoTrack clone:

```bash
git apply /path/to/Cursor/gotrack/fix_hydra_trainer.patch
```

Or copy these paths into the clone (same relative paths):

```text
utils/pl_compat.py
model/base.py
scripts/inference_gotrack.py
scripts/inference_pose_estimation.py
configs/machine/trainer/local.yaml
configs/machine/trainer/logger/tensorboard.yaml
configs/callback/lr/base.yaml
configs/callback/checkpoint/base.yaml
```

The shim:

- imports Trainer from `pytorch_lightning` or `lightning.pytorch`
- rewrites leftover `pytorch_lightning.*` Hydra targets
- on one GPU / CPU, drops `strategy: ddp` (upstream YAML always sets DDP)
- if CUDA is missing, switches `accelerator` to `cpu`

Then:

```bash
python -m scripts.inference_gotrack mode=pose_refinement dataset_name=lmo coarse_pose_method=foundpose
```

Set `root_dir` in `configs/user/default.yaml` first, as the GoTrack README requires.
