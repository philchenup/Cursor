# HWI / 哈焊 · 启动画面

科技简约启动界面，全部逻辑在 `src/main.cpp` 的 `main` 中。

- 画布 1080×540，平面深色底，无圆形光晕、无网格
- 四角 HUD 角标 + Logo（1490×468 透明 PNG）+ 细进度条
- 将正式 Logo 放到 `icon/preview.png`

```bash
cmake -S . -B build && cmake --build build
cd build && ./hwi_app
```

浏览器预览：`preview/splash-preview.html`
