# HWI 启动画面（Splash Screen）

为 **1490×468** 透明 PNG Logo 设计的 Qt 启动界面，含阶段性进度条与状态文案。

## 视觉结构

```
┌────────────── 1000 × 560 ──────────────┐
│         深蓝石板对角渐变 + 中心柔光        │
│                                        │
│              [ Logo ≤720 宽 ]           │
│            （保持 1490:468）             │
│                 ─────                   │
│              Version 1.0.0              │
│                                        │
│         状态文案              72%       │
│         ████████████░░░░░░              │
└────────────────────────────────────────┘
```

- **品牌优先**：Logo 为第一视口主视觉，无额外大标题抢戏
- **比例正确**：不再把 1490×468 强行拉伸到 1200×600
- **进度节奏**：前快 → 中稳 → 末缓，文案分 6 阶段切换

## 文件

| 路径 | 说明 |
|------|------|
| `include/SplashScreen.h` | 启动画面类声明 |
| `src/SplashScreen.cpp` | 背景绘制、布局、进度逻辑 |
| `src/main.cpp` | 集成示例 |
| `preview/splash-preview.html` | 浏览器可视化预览 |
| `icon/preview.png` | 放入你的透明 Logo（1490×468） |

## 使用

1. 将 Logo 保存为 `icon/preview.png`
2. 构建并运行：

```bash
cmake -S . -B build
cmake --build build
cd build && ./hwi_app
```

3. 或直接打开 `preview/splash-preview.html` 查看布局与动画

## 在现有工程中接入

```cpp
#include "SplashScreen.h"

SplashScreen splash("./icon/preview.png", "1.0.0");
splash.run();          // 阻塞至进度完成
// ... 创建主窗口 ...
splash.finish(&mainWindow);
```

也可在真实初始化流程中手动驱动：

```cpp
splash.show();
splash.setProgress(20, QString::fromUtf8(u8"初始化仿真引擎..."));
// ... 实际加载 ...
splash.setProgress(100);
```
