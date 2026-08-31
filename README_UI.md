# NexusVIT Command Workspace UI

原界面是典型 CAD 工作站：顶栏大图标 + 左右堆叠 GroupBox + 白底三维视口。
新界面改成 **视口优先的 Command Workspace**，所有按钮和输入仍在，只换布局与视觉语言。

## 设计原则

1. **不改功能入口**：按钮文案、控件 `objectName` 与现有 `on_<name>_clicked` 槽保持一致，QMetaObject 自动连接仍然有效。
2. **背景仍用炭灰**：`#1c1e22` / `#22252a`，接近原 `#2B2B2B`。
3. **结构完全不同**：
   - 顶栏：品牌 + 文字菜单 + 状态胶囊（不再用一排彩色大图标当主视觉）
   - 命令条：按「文件 / 点云 / 标定 / 运行」分组的线框小按钮
   - 工作区：左数据树、中深色视口、右属性表
   - 底部 **控制台甲板**：机器人 / 视觉 / 工具 / 通信 / 控制台 五个页签  
     （原左右各 4+2 个堆叠面板全部迁到这里）

## 怎么接到现有工程

只换皮肤：

```cpp
#include "NexusWorkspaceWindow.h"
ApplyNexusWorkspaceTheme(this);   // setupUi() 之后
```

换布局：用 `NexusWorkspaceWindow` 作为主窗口骨架，或把 `ui/nexus_workspace.qss` 套到现有 `QMainWindow`，再把左右 Dock 收进底部 `QTabWidget`（见 `NexusWorkspaceWindow::buildDeck`）。

## 预览

- 交互原型：打开 `preview/nexusvit-workspace.html`
- Qt 预览（需 Qt 5.15+ / 6）：

```bash
cmake -S . -B build
cmake --build build
./build/nexusvit_ui_preview
```

原界面截图见 `preview/original-ui.png`。
