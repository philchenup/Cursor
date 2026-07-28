# Vosk SSL model-download fix

Official Vosk `test_microphone.py` auto-downloads models with `urllib.request.urlretrieve`.
On many Windows Python environments that fails:

```text
vosk-model-small-nl-0.22.zip: 0.00B [00:00, ?B/s]
SSLError: [ASN1: NOT_ENOUGH_DATA] not enough data (_ssl.c:4040)
AttributeError: 'Model' object has no attribute '_handle'
```

Root cause:
1. Primary: SSL failure while downloading from `https://alphacephei.com/vosk/models/`
2. Secondary: `Model.__init__` aborts before setting `_handle`, then `__del__` crashes

This folder replaces `urlretrieve` with `requests` (certifi CA bundle), which is the
approach recommended in [vosk-api#1456](https://github.com/alphacep/vosk-api/issues/1456).

## Install

```bash
pip install vosk sounddevice requests tqdm
```

## Option A — use the fixed microphone example (recommended)

```bash
cd vosk_fix
python test_microphone.py -m nl
```

Or point at a manually downloaded model:

```bash
python test_microphone.py --model-path "%USERPROFILE%\AppData\Local\vosk\vosk-model-small-nl-0.22"
```

## Option B — download the model only

```bash
cd vosk_fix
python download_model.py -l nl
```

Then run the official example with a local path:

```python
from vosk import Model
model = Model(model_path=r"C:\Users\<you>\AppData\Local\vosk\vosk-model-small-nl-0.22")
```

## Option C — patch the installed vosk package in-process

```python
from patch_vosk import apply
apply()

from vosk import Model
model = Model(lang="nl")  # now uses requests for download
```

## Manual download fallback

1. Open https://alphacephei.com/vosk/models
2. Download `vosk-model-small-nl-0.22.zip`
3. Extract into `%USERPROFILE%\AppData\Local\vosk\` (Windows) or `~/.cache/vosk/` (Linux/macOS)
4. Load with `Model(model_path=...)` — never rely on auto-download if SSL keeps failing

## Files

| File | Purpose |
|------|---------|
| `download_model.py` | SSL-safe downloader + monkey-patch helpers |
| `test_microphone.py` | Fixed microphone demo |
| `patch_vosk.py` | One-liner patch for existing scripts |
