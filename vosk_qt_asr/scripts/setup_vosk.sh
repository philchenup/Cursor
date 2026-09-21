#!/usr/bin/env bash
# 下载 Vosk 预编译库到 third_party/vosk/ 与 include/
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
VERSION="${VOSK_VERSION:-0.3.45}"
ARCH="$(uname -m)"

case "${ARCH}" in
  x86_64|amd64)  PKG="vosk-linux-x86_64-${VERSION}" ;;
  aarch64|arm64) PKG="vosk-linux-aarch64-${VERSION}" ;;
  armv7l)        PKG="vosk-linux-armv7l-${VERSION}" ;;
  *)
    echo "不支持的架构: ${ARCH}。Windows 请手动下载 vosk-win64 包。"
    exit 1
    ;;
esac

URL="https://github.com/alphacep/vosk-api/releases/download/v${VERSION}/${PKG}.zip"
TMP="$(mktemp -d)"
trap 'rm -rf "${TMP}"' EXIT

echo "下载 ${URL} ..."
curl -fL --progress-bar -o "${TMP}/vosk.zip" "${URL}"
unzip -qo "${TMP}/vosk.zip" -d "${TMP}"

mkdir -p "${ROOT}/third_party/vosk" "${ROOT}/include"
cp "${TMP}/${PKG}/libvosk.so" "${ROOT}/third_party/vosk/"
cp "${TMP}/${PKG}/vosk_api.h" "${ROOT}/include/"

echo "已安装:"
echo "  ${ROOT}/third_party/vosk/libvosk.so"
echo "  ${ROOT}/include/vosk_api.h"
