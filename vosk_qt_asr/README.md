# Vosk 中文语音识别（Visual Studio + Qt）

基于 [vosk-api](https://github.com/alphacep/vosk-api) 的离线中文识别示例：

- 界面中央有一个**音响按钮**
- **第一次点击**：开始录音
- **再次点击**：结束录音，并将识别文字显示在结果区（同时输出到控制台 / VS 输出窗口）

## 目录结构

```text
vosk_qt_asr/
├── CMakeLists.txt          # Visual Studio / CMake 推荐
├── vosk_qt_asr.pro         # Qt Creator / Qt VS Tools 可选
├── include/
│   ├── audio_capture.h     # Qt Multimedia 麦克风采集
│   ├── vosk_asr_engine.h   # Vosk 识别封装
│   ├── main_window.h       # 主窗口（音响按钮）
│   └── vosk_api.h
├── src/
│   ├── main.cpp
│   ├── audio_capture.cpp
│   ├── vosk_asr_engine.cpp
│   └── main_window.cpp
├── model/                  # 放置中文模型（已下载则直接用）
├── third_party/vosk/       # libvosk.dll / .lib / .so
└── scripts/
```

## 依赖

| 组件 | 说明 |
|------|------|
| Visual Studio 2019/2022 | 含 C++ 桌面开发工作负载 |
| Qt 5.15+ 或 Qt 6 | 组件：`Widgets`、`Multimedia` |
| Vosk 预编译库 | [Releases](https://github.com/alphacep/vosk-api/releases) |
| 中文模型 | 放到 `model/`，如 `vosk-model-small-cn-0.22` |

## 准备模型与库

### 1. 中文模型

若模型已下载，解压**内容**到 `vosk_qt_asr/model/`（目录内应有 `am/`、`conf/`、`graph/` 等，不要多套一层文件夹名）。

也可：

```bash
cd vosk_qt_asr
bash scripts/download_cn_model.sh
```

### 2. Vosk 库（Windows / Visual Studio）

1. 下载 [vosk-win64-0.3.45.zip](https://github.com/alphacep/vosk-api/releases/download/v0.3.45/vosk-win64-0.3.45.zip)
2. 将文件放到：

```text
include/vosk_api.h
third_party/vosk/libvosk.dll
third_party/vosk/libvosk.lib
```

CMake 构建后会把 `libvosk.dll` 复制到 exe 旁。

### 3. Linux

```bash
bash scripts/setup_vosk.sh
```

## Visual Studio 打开方式（推荐 CMake）

1. 安装 **Qt VS Tools**，并在扩展中配置 Qt 版本路径
2. **文件 → 打开 → CMake…**，选择 `vosk_qt_asr/CMakeLists.txt`
3. 或用开发者命令行：

```bat
cd vosk_qt_asr
cmake -S . -B build -G "Visual Studio 17 2022" -A x64 ^
  -DCMAKE_PREFIX_PATH=C:/Qt/6.5.3/msvc2019_64
cmake --build build --config Release
```

4. 调试前设置**工作目录**为 `vosk_qt_asr`（能找到 `model/`），或把 `model` 拷到 exe 同级
5. F5 运行 → 点「开始录音」→ 说话 → 再点「结束录音」→ 查看识别结果

也可用 `vosk_qt_asr.pro`：Qt VS Tools → Open Qt Project File (.pro)。

## Linux / macOS 命令行构建

```bash
cd vosk_qt_asr
cmake -S . -B build -DCMAKE_PREFIX_PATH=/usr/lib/x86_64-linux-gnu/cmake/Qt6
cmake --build build -j
./build/vosk_qt_asr -m model
```

## 操作说明

1. 启动后状态栏显示模型是否就绪
2. 点击圆形**开始录音**按钮 → 对着麦克风说话
3. 再次点击**结束录音** → 结果区打印识别文字
4. 录音过程中状态栏会显示中间结果；静音切分出的完整句会即时追加

## 核心调用链

```text
音响按钮点击
  → AudioCapture (16kHz/16bit/mono PCM)
  → VoskAsrEngine::writeAudio / endUtterance
  → vosk_recognizer_accept_waveform / final_result
  → MainWindow 结果文本框 + qInfo 控制台输出
```

## 参考

- https://github.com/alphacep/vosk-api
- 模型列表：https://alphacephei.com/vosk/models
