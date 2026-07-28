# Cursor

## Vosk SSL download fix

If `vosk` `test_microphone` fails with:

```text
SSLError: [ASN1: NOT_ENOUGH_DATA] not enough data (_ssl.c:4040)
AttributeError: 'Model' object has no attribute '_handle'
```

see **[vosk_fix/README.md](vosk_fix/README.md)**.

Quick start:

```bash
cd vosk_fix
pip install -r requirements.txt
python test_microphone.py -m nl
```
