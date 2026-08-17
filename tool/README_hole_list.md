# CAD 孔位列表（holeListWidget）

`holes = detector.holes()` 检测完成后，会像 `graspListWidget` 一样把每个孔位填进列表控件。

## Qt Designer

在 CAD 页面增加一个 `QListWidget`，`objectName` 设为：

```text
holeListWidget
```

建议放在 `cadqvtkWidget` 旁侧。若未在 `.ui` 中添加，代码会自动 `findChild`；找不到时会在 CAD 视图父布局旁创建一个备用列表。

## 交互

- 列表项：`Hole_1 (x, y, z)` …，样式与 `graspListWidget` 一致
- 单击：选中项对应坐标系放大高亮
- 右键：
  - **添加为抓取点**：写入 `graspListWidget`（复用 `autoAddGraspPoint`）
  - **Delete**：从当前 `holes` 中移除并刷新显示

## 接入点

在 `on_handExecBtn_clicked` 中：

```cpp
holes = detector.holes();
updateHoleListWidget();
```
