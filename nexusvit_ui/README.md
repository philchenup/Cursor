# NexusVIT UI Shell

Qt 6 深色工业风界面，对照 NexusVIT UNORDERED GRASP 截图还原布局与风格。不含业务逻辑。

## 布局

- 顶栏：Logo、菜单、Robot / Vision / Communication / Tool 状态胶囊
- 功能区：文件、点云、标定，以及单步 / 连续 / 暂停 / 复位
- 左：Data 树、Properties 表
- 中：Scene / Camera 视口（Scene 可拖拽旋转、滚轮缩放）
- 右：Vision、Robot 面板
- 底：Tool / Communication / Console

## 运行

```bash
cmake -S nexusvit_ui -B nexusvit_ui/build -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER=g++
cmake --build nexusvit_ui/build -j
./nexusvit_ui/build/nexusvit_ui
```
