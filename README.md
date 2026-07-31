# HWI / 哈焊 · 启动画面

无网格科技简约启动界面，已融合进工程 `main`（单例检测 / VTK / MainWindow）。

核心文件：`src/main.cpp`

- 画布 1080×540，平面深色底，无圆形光晕、无网格
- 四角 HUD 角标 + Logo（`./icon/preview.png`，1490×468）+ 细进度条
- 保留原有：单实例、`EventFilter`、metaType 注册、`MainWindow` 启动流程

将正式 Logo 放到 `icon/preview.png`。浏览器预览：`preview/splash-preview.html`
