# GoTrack Windows + Hydra `pytorch_lightning.Trainer` fix

Drop-in for [facebookresearch/gotrack](https://github.com/facebookresearch/gotrack).

## Error

```text
Error executing job with overrides: []
Error locating target 'pytorch_lightning.Trainer', set env var HYDRA_FULL_ERROR=1 to see chained exception.
full_key: machine.trainer
```

Hydra instantiates `configs/machine/trainer/local.yaml` (`_target_: pytorch_lightning.Trainer`). That import fails when `pytorch-lightning` is missing, only Lightning 2 (`lightning`) is installed, or the package crashes on import.

On **Windows** the same command then hits Linux-only defaults:

| Upstream default | Windows result |
| --- | --- |
| `PYOPENGL_PLATFORM=egl` | pyrender/OpenGL crash (EGL is Linux) |
| `strategy: ddp` | NCCL is not shipped with PyTorch on Windows |
| `DataLoader(num_workers=1)` / `multiprocessing.Pool` | spawn + Hydra re-enters `main` |
| `root_dir: D:\data\...` in YAML | `\d` is an escape, path is wrong |

## Windows setup

Use **Python 3.10**. Official `environment.yml` pulls Linux-only packages (`faiss-gpu`, `cuml-cu11`, `xformers`, `scripts/env.sh`).

1. Clone GoTrack and apply this drop-in (see below).
2. Install CPU or CUDA PyTorch from https://pytorch.org (Windows).
3. Install the rest:

```bat
cd C:\path\to\gotrack
python -m pip install -r C:\path\to\Cursor\gotrack\requirements-windows.txt
cd external\bop_toolkit
python -m pip install -e .
cd ..\dinov2
python -m pip install -e .
cd ..\..
```

4. Edit `configs/user/default.yaml`. Use **forward slashes**:

```yaml
project_name: gotrack
root_dir: D:/gotrack_project
```

5. Put the pretrained `gotrack_checkpoint.pt` where the model config expects it.

6. Run from the GoTrack repo root (cmd.exe):

```bat
set HYDRA_FULL_ERROR=1
set PYTHONPATH=%CD%;%PYTHONPATH%
python -m scripts.inference_gotrack mode=pose_refinement dataset_name=lmo coarse_pose_method=foundpose
```

PowerShell:

```powershell
$env:HYDRA_FULL_ERROR = "1"
$env:PYTHONPATH = "$PWD;$env:PYTHONPATH"
python -m scripts.inference_gotrack mode=pose_refinement dataset_name=lmo coarse_pose_method=foundpose
```

Or copy `run_inference_gotrack.bat` to the GoTrack root and:

```bat
run_inference_gotrack.bat mode=pose_refinement dataset_name=lmo coarse_pose_method=foundpose
```

If CUDA is installed, the shim keeps `accelerator=gpu` and drops DDP. If not, it switches to CPU.

## Apply this drop-in

From a GoTrack clone:

```bat
git apply C:\path\to\Cursor\gotrack\fix_hydra_trainer.patch
```

Or copy these paths into the clone (same relative paths):

```text
utils/pl_compat.py
utils/win_compat.py
utils/renderer.py
utils/template_util.py
utils/misc.py
model/base.py
scripts/inference_gotrack.py
scripts/inference_pose_estimation.py
configs/machine/trainer/local.yaml
configs/machine/trainer/logger/tensorboard.yaml
configs/callback/lr/base.yaml
configs/callback/checkpoint/base.yaml
```

## Linux (original error only)

```bash
pip install 'pytorch-lightning==1.8.6' hydra-core==1.3.2
export HYDRA_FULL_ERROR=1
python -m scripts.inference_gotrack mode=pose_refinement dataset_name=lmo coarse_pose_method=foundpose
```
