# NexusVIT Web 工作台

原界面是 Qt 工业 CAD 布局：顶栏彩色大图标、左右堆叠 GroupBox、白底三维视口、底部控制台。

新界面用 **Web** 重做皮肤，做成视口优先的深色工业工作台。**所有原功能入口都在**，只换布局与视觉，不改控制 / 视觉 / 通信语义。

## 打开预览

```bash
python3 -m http.server 8080 --directory web
```

浏览器打开 `http://127.0.0.1:8080/`。

效果图：`preview/nexusvit-web-ui-mockup.png`  
原界面：`preview/original-ui.png`

## 设计原则

1. **功能一一对应**：打开 / 保存 / 删除、点云处理、TCP 与手眼标定、工件库、视觉配置、摆放、单步 / 连续 / 暂停 / 重置、机器人 IP·关节·笛卡尔、GoCap / GoHom、视觉 Cap / Connect、工具 Screw、TCP 通信、控制台日志，全部保留。
2. **位置可预期**：数据与属性仍在左侧；视觉与机器人仍在右侧（原右侧面板）；工具、通信、控制台收到底部甲板，给三维视口让出高度。
3. **视觉语言**：炭灰底 `#0c1118`、薄荷绿连接态、珊瑚红对应原红白协作臂。数字用等宽字体。
4. **三维视口**：Three.js 实时渲染 6 轴机械臂、工作台、无序料框。`GoHom` / `GoCap` / 连续运行会带动画，关节与笛卡尔读数同步刷新。

## 布局对照

| 原 Qt 界面 | Web 工作台 |
| --- | --- |
| 菜单 文件…帮助 | 顶栏同一套中文菜单 |
| 彩色大图标工具条 | 分组命令条：文件 / 点云 / 标定 / 运行 |
| 左：数据、属性、工具、通信 | 左：数据 + 属性；工具 / 通信改到底部页签 |
| 中：Scene / Camera | 中：WebGL Scene + Camera 实况遮罩 |
| 右：视觉、机器人 | 右：视觉卡片 + 机器人卡片 |
| 底：控制台 | 底：工具 / 通信 / 控制台甲板，可折叠 |

## 接到现有工程

Web 页是交互规格与预览。现有 Qt 槽函数（`on_<objectName>_clicked`）不要改名；把本界面的按钮 `id` 对齐原 `objectName` 即可继续自动连接。本预览里已使用原名，例如：

`robotConnectBtn` · `InitializeRobot` · `cam_btn_connect` · `GoCapBtn` · `setHomBtn` · `scoketComb` · `cloudtree` 对应的数据树。
