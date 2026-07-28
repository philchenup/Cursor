"""Apply SSL-safe download monkey-patch to an installed vosk package.

Usage (once per Python process, before Model(...)):

  from patch_vosk import apply
  apply()

  from vosk import Model
  model = Model(lang="nl")
"""

from download_model import patch_vosk_download


def apply() -> None:
    patch_vosk_download()


if __name__ == "__main__":
    apply()
    print("Patched vosk.Model.download_model to use requests.")
    print("Import this module (or call apply()) before creating Model(lang=...).")
