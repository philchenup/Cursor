# Vosk C++ microphone ASR (aligned with Python)

Fixed `voice_ASR_1200.cpp` so recognition matches the Python `sounddevice` + Vosk mic pipeline.

## What was wrong

| Issue | Old C++ | Python / fixed C++ |
|---|---|---|
| Waveform API | `float * 32768` then `accept_waveform_f` | int16 PCM + `accept_waveform_s` / `AcceptWaveform` |
| Sample rate | Hardcoded 16000, may disagree with device | Device rate passed into recognizer |
| Channels | Treated interleaved float as mono | Take channel 0 only (`channels=1`) |
| Buffering | Shared `ptr` / race / drop frames | Thread-safe queue (like Python `queue`) |

## Usage

1. Put the same model directory Python uses, e.g. `model/vosk-model-cn-kaldi-multicn-0.15`.
2. Build against [vosk-api](https://github.com/alphacep/vosk-api) and C++ audio (`std::experimental::audio` / P1386).
3. Run:

```bash
./voice_ASR_1200
./voice_ASR_1200 /path/to/vosk-model-cn-kaldi-multicn-0.15
```

4. Controls:
   - Press **S** to start recording
   - Press **ESC** to stop recording and print the final recognition result

5. Post-process: **any** Chinese numerals in the result are converted to Arabic digits
   via `normalize_chinese_numbers()` — including 十/百/千/万/亿/兆、负号、小数点、
   大写数字、全角数字、廿/卅，以及 ASR 插入的空格（`十二毫米` / `十 二 毫 米` → `12毫米`).

```bash
# unit test for number normalization (no mic / vosk needed)
c++ -std=c++17 -O2 -o test_chinese_number \
  chinese_number.cpp test_chinese_number.cpp
./test_chinese_number
```

## Match Python exactly

- Use the **same model directory**.
- Keep recognizer sample rate equal to the mic stream sample rate.
- Feed **mono int16** only (do not scale and call `_f`).
