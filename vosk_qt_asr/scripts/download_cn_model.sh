#!/usr/bin/env bash
# 下载中文模型到 model/（若目录尚未包含模型文件）
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
MODEL_DIR="${ROOT}/model"
MODEL_NAME="${VOSK_CN_MODEL:-vosk-model-small-cn-0.22}"
URL="https://alphacephei.com/vosk/models/${MODEL_NAME}.zip"

if [[ -f "${MODEL_DIR}/am/final.mdl" || -f "${MODEL_DIR}/conf/model.conf" || -d "${MODEL_DIR}/graph" ]]; then
  echo "检测到 model/ 已有模型，跳过下载。"
  exit 0
fi

TMP="$(mktemp -d)"
trap 'rm -rf "${TMP}"' EXIT

echo "下载中文模型 ${MODEL_NAME} ..."
curl -fL --progress-bar -o "${TMP}/model.zip" "${URL}"
unzip -qo "${TMP}/model.zip" -d "${TMP}"

SRC="${TMP}/${MODEL_NAME}"
if [[ ! -d "${SRC}" ]]; then
  SRC="$(find "${TMP}" -mindepth 1 -maxdepth 1 -type d | head -1)"
fi

mkdir -p "${MODEL_DIR}"
find "${MODEL_DIR}" -mindepth 1 -maxdepth 1 ! -name '.gitkeep' -exec rm -rf {} +
cp -a "${SRC}/." "${MODEL_DIR}/"

echo "模型已解压到: ${MODEL_DIR}"
ls "${MODEL_DIR}"
