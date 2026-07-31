# HWI / 哈焊 · 启动画面

按 **CAM + HWI** 双色标（1490×468 透明 PNG）设计的 Qt 启动界面。

## 品牌对齐

| 色 | Hex | 用途 |
|----|-----|------|
| 蓝 | `#0070D2` | Logo 主色、进度条起点、右侧柔光 |
| 橙 | `#F39C12` | CAM 三角、进度条末端、左侧柔光、装饰线 |
| 红 | `#DC2626` | Logo 内「哈焊」（界面不再重复绘制） |
| 深炭 | `#0A1018 → #152436` | 背景，突出彩色 Logo |

## 布局（1080 × 540）

科技简约：无圆形光晕，细网格 + 四角 HUD 角标。

```
┌──────────────────────────────────────────┐
│ ▢ 细网格深色底 · 四角角标                  │
│                                          │
│         [ CAM 标 + HWI 椭圆 Logo ]        │
│                 细分割线                  │
│               Version 1.0.0              │
│                                          │
│      状态文案                    52%      │
│      ████████░░░░░░                      │
└──────────────────────────────────────────┘
```

- Logo 最大宽 **860**，`KeepAspectRatio`，不再拉成 1200×600
- **不**再叠加红色「HWI」大标题（Logo 已含品牌字）
- 进度：前快 → 中稳 → 末缓，6 段中文状态

## 使用

将真实 Logo 覆盖为 `icon/preview.png`（1490×468、透明），然后：

```bash
cmake -S . -B build && cmake --build build
cd build && ./hwi_app
```

浏览器预览：打开 `preview/splash-preview.html`

```cpp
SplashScreen splash("./icon/preview.png", "1.0.0");
splash.run();
// 或 splash.setProgress(n, status);
```
