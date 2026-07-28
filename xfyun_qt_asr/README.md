# 科大讯飞语音识别 + Qt 对话框

将讯飞 MSC（语音听写）C++ SDK 集成到 Qt 对话框中，**同时支持语音输入与键盘打字**。

## 功能

- `QPlainTextEdit` 支持直接打字、编辑、粘贴
- 「开始说话 / 结束说话」调用讯飞在线听写（IAT）
- 麦克风采集：16 kHz / 16 bit / 单声道 PCM
- 中间结果实时回填，最终结果写入同一文本框
- 默认提供 **Mock MSC**，无正式 SDK 也可先编译跑通 UI 与录音链路

## 目录结构

```text
xfyun_qt_asr/
├── CMakeLists.txt
├── config/xfyun.env.example
├── include/
│   ├── audio_capture.h          # 麦克风采集
│   ├── voice_input_dialog.h     # 语音+打字对话框
│   └── xfyun_asr_engine.h       # 讯飞听写封装
├── src/
│   ├── main.cpp
│   ├── audio_capture.cpp
│   ├── voice_input_dialog.cpp
│   ├── xfyun_asr_engine.cpp
│   └── msc_mock.cpp             # 无正式库时的本地模拟
└── third_party/xfyun/
    ├── include/                 # 与官方 API 对齐的头文件
    └── lib/                     # 放置官方 libmsc / msc_x64.lib
```

## 依赖

- CMake ≥ 3.16
- C++17
- Qt5 或 Qt6：`Widgets`、`Multimedia`
- 正式运行还需：[讯飞开放平台](https://www.xfyun.cn/) APPID + 对应平台 MSC SDK

## 快速开始（Mock，无需正式 SDK）

```bash
cd xfyun_qt_asr
cmake -S . -B build -DXFYUN_USE_MOCK=ON
cmake --build build -j
export XFYUN_APPID=demo
./build/xfyun_qt_asr --appid demo
```

对话框中可直接打字；点击「开始说话」会走 Mock 识别，结束说话后写入模拟文本。

## 接入正式讯飞 SDK

1. 在控制台创建应用，开通 **语音听写（流式版）**，记下 `APPID`
2. 下载对应平台 SDK，将内容放到：

```text
third_party/xfyun/
  include/   # 可用官方头文件覆盖本仓库自带头文件
  lib/       # Linux: libmsc.so ; Windows: msc_x64.lib / msc.lib
  bin/       # Windows: msc_x64.dll 及 msc 资源目录
```

3. 关闭 Mock 并编译：

```bash
cmake -S . -B build -DXFYUN_USE_MOCK=OFF
cmake --build build -j
export XFYUN_APPID=你的APPID
./build/xfyun_qt_asr --appid 你的APPID
```

4. Windows 请将 `msc_x64.dll` 与 `msc/` 资源目录放到可执行文件同级或工作目录。

## 在自己的 Qt 工程中嵌入对话框

```cpp
#include "voice_input_dialog.h"

void MainWindow::onOpenInput()
{
    VoiceInputDialog dlg(qEnvironmentVariable("XFYUN_APPID"), this);
    if (dlg.exec() == QDialog::Accepted) {
        const QString text = dlg.text();  // 语音识别 + 手动编辑后的最终文本
        // 使用 text ...
    }
}
```

核心调用链：

```text
麦克风 PCM → AudioCapture::pcmReady
           → XfyunAsrEngine::writeAudio
           → QISRAudioWrite / QISRGetResult
           → partialResult / finalResult
           → VoiceInputDialog 文本框
```

## 听写会话参数说明

`XfyunAsrEngine` 默认会话参数：

```text
sub = iat, domain = iat, language = zh_cn, accent = mandarin,
sample_rate = 16000, result_type = plain, result_encoding = utf8,
vad_eos = 3000, vad_bos = 5000, ptt = 1
```

可按业务在 `xfyun_asr_engine.cpp` 的 `buildSessionParams()` 中调整。

## 注意事项

- APPID 不要硬编码进仓库，优先用环境变量 `XFYUN_APPID` 或命令行 `--appid`
- 音频格式必须与 `sample_rate` 一致，否则识别失败或乱码
- 正式 SDK 的头文件若与本仓库 stub 冲突，以官方包为准覆盖 `third_party/xfyun/include`
- Linux 需保证运行时能找到 `libmsc.so`（RPATH 已指向 `third_party/xfyun/lib|bin`）
