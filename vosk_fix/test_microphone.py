#!/usr/bin/env python3
"""Fixed microphone example for Vosk.

Why this exists
---------------
Official `test_microphone.py` calls `Model(lang=...)`, which auto-downloads via
`urllib.request.urlretrieve`. On many Windows Python installs that fails with:

  SSLError: [ASN1: NOT_ENOUGH_DATA] not enough data (_ssl.c:4040)

and then a secondary:

  AttributeError: 'Model' object has no attribute '_handle'

This script downloads with `requests` (certifi) first, then loads the model by
local path so the broken urlretrieve path is never used.

Prerequisites
-------------
  pip install vosk sounddevice requests tqdm

Usage
-----
  python test_microphone.py -m nl
  python test_microphone.py --model-path D:/models/vosk-model-small-nl-0.22
  python test_microphone.py -l   # list audio devices
"""

from __future__ import annotations

import argparse
import queue
import sys
from pathlib import Path

import sounddevice as sd
from vosk import KaldiRecognizer, Model

from download_model import ensure_model, patch_vosk_download

q: queue.Queue[bytes] = queue.Queue()


def int_or_str(text: str):
    try:
        return int(text)
    except ValueError:
        return text


def callback(indata, frames, time, status):  # noqa: ANN001, ARG001
    if status:
        print(status, file=sys.stderr)
    q.put(bytes(indata))


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Microphone speech recognition with a SSL-safe Vosk model download",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "-l",
        "--list-devices",
        action="store_true",
        help="show list of audio devices and exit",
    )
    parser.add_argument(
        "-f",
        "--filename",
        type=str,
        metavar="FILENAME",
        help="audio file to store recording to",
    )
    parser.add_argument(
        "-d",
        "--device",
        type=int_or_str,
        help="input device (numeric ID or substring)",
    )
    parser.add_argument("-r", "--samplerate", type=int, help="sampling rate")
    parser.add_argument(
        "-m",
        "--model",
        type=str,
        default=None,
        help="language model code; e.g. en-us, fr, nl; default is en-us",
    )
    parser.add_argument(
        "--model-path",
        type=Path,
        default=None,
        help="local path to an already downloaded/extracted Vosk model directory",
    )
    parser.add_argument(
        "--model-name",
        type=str,
        default=None,
        help="exact model folder name, e.g. vosk-model-small-nl-0.22",
    )
    return parser


def load_model(args: argparse.Namespace) -> Model:
    """Load model without using vosk's broken urlretrieve downloader."""
    # Patch for any accidental Model(lang=...) usage elsewhere in the process.
    patch_vosk_download()

    if args.model_path is not None:
        path = Path(args.model_path)
        if not path.is_dir():
            raise SystemExit(f"model path does not exist: {path}")
        return Model(model_path=str(path))

    path = ensure_model(lang=args.model or "en-us", model_name=args.model_name)
    return Model(model_path=str(path))


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()

    if args.list_devices:
        print(sd.query_devices())
        return

    try:
        if args.samplerate is None:
            device_info = sd.query_devices(args.device, "input")
            args.samplerate = int(device_info["default_samplerate"])

        model = load_model(args)

        dump_fn = open(args.filename, "wb") if args.filename else None

        with sd.RawInputStream(
            samplerate=args.samplerate,
            blocksize=8000,
            device=args.device,
            dtype="int16",
            channels=1,
            callback=callback,
        ):
            print("#" * 80)
            print("Press Ctrl+C to stop the recording")
            print("#" * 80)

            rec = KaldiRecognizer(model, args.samplerate)
            while True:
                data = q.get()
                if rec.AcceptWaveform(data):
                    print(rec.Result())
                else:
                    print(rec.PartialResult())
                if dump_fn is not None:
                    dump_fn.write(data)

    except KeyboardInterrupt:
        print("\nDone")
    except Exception as exc:  # noqa: BLE001
        print(f"{type(exc).__name__}: {exc}", file=sys.stderr)
        raise SystemExit(1) from exc
    finally:
        if "dump_fn" in locals() and dump_fn is not None:
            dump_fn.close()


if __name__ == "__main__":
    main()
