#!/usr/bin/env python3
"""Robust Vosk model downloader.

Vosk's built-in download uses urllib.request.urlretrieve, which often fails on
Windows with SSL errors such as:
  SSLError: [ASN1: NOT_ENOUGH_DATA] not enough data (_ssl.c:4040)
  SSLCertVerificationError: certificate verify failed

This helper downloads with the requests library (uses certifi CA bundle) and
caches models under ~/.cache/vosk (or %LOCALAPPDATA%/vosk on Windows).
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from zipfile import ZipFile

import requests
from tqdm import tqdm

MODEL_PRE_URL = "https://alphacephei.com/vosk/models/"
MODEL_LIST_URL = MODEL_PRE_URL + "model-list.json"


def default_model_dir() -> Path:
    """Match vosk's preferred cache locations."""
    import os

    env = os.getenv("VOSK_MODEL_PATH")
    if env:
        return Path(env)

    if sys.platform == "win32":
        local = Path.home() / "AppData" / "Local" / "vosk"
        return local

    return Path.home() / ".cache" / "vosk"


def fetch_model_list(timeout: int = 30) -> list[dict]:
    response = requests.get(MODEL_LIST_URL, timeout=timeout)
    response.raise_for_status()
    return response.json()


def resolve_model_name(lang: str | None = None, model_name: str | None = None) -> str:
    models = fetch_model_list()

    if model_name:
        matches = [m["name"] for m in models if m["name"] == model_name]
        if not matches:
            raise SystemExit(f"model name {model_name!r} does not exist")
        return matches[0]

    if not lang:
        lang = "en-us"

    matches = [
        m["name"]
        for m in models
        if m["lang"] == lang and m["type"] == "small" and m["obsolete"] == "false"
    ]
    if not matches:
        raise SystemExit(f"lang {lang!r} does not exist (or has no small model)")
    return matches[0]


def download_file(url: str, dest: Path, chunk_size: int = 1024 * 256) -> None:
    dest.parent.mkdir(parents=True, exist_ok=True)
    partial = dest.with_suffix(dest.suffix + ".partial")

    with requests.get(url, stream=True, timeout=60) as response:
        response.raise_for_status()
        total = int(response.headers.get("content-length", 0)) or None
        with (
            open(partial, "wb") as out,
            tqdm(
                total=total,
                unit="B",
                unit_scale=True,
                unit_divisor=1024,
                desc=dest.name,
                miniters=1,
            ) as bar,
        ):
            for chunk in response.iter_content(chunk_size=chunk_size):
                if not chunk:
                    continue
                out.write(chunk)
                bar.update(len(chunk))

    partial.replace(dest)


def ensure_model(
    lang: str | None = None,
    model_name: str | None = None,
    model_dir: Path | None = None,
) -> Path:
    """Download (if needed) and return the local model directory path."""
    cache = model_dir or default_model_dir()
    cache.mkdir(parents=True, exist_ok=True)

    name = resolve_model_name(lang=lang, model_name=model_name)
    model_path = cache / name
    if model_path.is_dir() and any(model_path.iterdir()):
        print(f"Using existing model: {model_path}")
        return model_path

    zip_path = cache / f"{name}.zip"
    url = MODEL_PRE_URL + f"{name}.zip"
    print(f"Downloading {url}")
    try:
        download_file(url, zip_path)
    except requests.exceptions.SSLError as exc:
        raise SystemExit(
            "SSL download failed. Try:\n"
            "  1) pip install -U certifi requests\n"
            "  2) Or manually download the zip from https://alphacephei.com/vosk/models\n"
            f"     extract it into {cache}\n"
            f"Original error: {exc}"
        ) from exc
    except requests.RequestException as exc:
        raise SystemExit(f"Failed to download model: {exc}") from exc

    print(f"Extracting to {cache}")
    with ZipFile(zip_path, "r") as zf:
        zf.extractall(cache)
    zip_path.unlink(missing_ok=True)

    if not model_path.is_dir():
        raise SystemExit(f"Expected model directory missing after extract: {model_path}")

    return model_path


def patch_vosk_download() -> None:
    """Monkey-patch vosk.Model.download_model to use requests instead of urlretrieve."""
    from vosk import Model

    def download_model(self, model_name: Path) -> None:  # noqa: ANN001
        parent = model_name.parent
        parent.mkdir(parents=True, exist_ok=True)
        zip_path = Path(str(model_name) + ".zip")
        url = MODEL_PRE_URL + str(model_name.name) + ".zip"
        download_file(url, zip_path)
        with ZipFile(zip_path, "r") as model_ref:
            model_ref.extractall(parent)
        zip_path.unlink(missing_ok=True)

    Model.download_model = download_model

    # Avoid AttributeError in Model.__del__ when init fails before _handle is set
    _orig_del = Model.__del__

    def __del__(self):  # noqa: ANN001, N807
        if getattr(self, "_handle", None) is None:
            return
        return _orig_del(self)

    Model.__del__ = __del__


def main() -> None:
    parser = argparse.ArgumentParser(description="Download a Vosk speech model")
    parser.add_argument("-l", "--lang", default=None, help="language code, e.g. nl, en-us")
    parser.add_argument("-n", "--name", default=None, help="exact model name")
    parser.add_argument(
        "-d",
        "--dir",
        type=Path,
        default=None,
        help="model cache directory (default: vosk cache path)",
    )
    args = parser.parse_args()
    path = ensure_model(lang=args.lang, model_name=args.name, model_dir=args.dir)
    print(path)


if __name__ == "__main__":
    main()
