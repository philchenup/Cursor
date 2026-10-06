# Cursor

## ScaleAISShapeBy1000

将 OpenCASCADE 的 `AIS_Shape*` 缩小 1000 倍，并返回一个新的 `AIS_Shape*`。

```cpp
#include "ScaleAISShape.h"

AIS_Shape* ais = /* 已有对象 */;
AIS_Shape* scaled = ScaleAISShapeBy1000(ais);

// 推荐用 Handle 接管返回值，避免泄漏
Handle(AIS_Shape) scaledHandle = ScaleAISShapeBy1000(ais);
```

## WeldListWidget

焊缝工艺表：追加焊缝、按内缩刷新起终点、清空列表，以及在选中行时把起点和终点读成 `Eigen::Vector3d`。单击表格空白处取消选中。

```cpp
#include "WeldListWidget.h"

QTableWidget* table = setupWeldListWidget(dock, {
    { Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(100, 0, 0) },
});

setWeldSelectionCallback(table,
    [](int row, const Eigen::Vector3d& start, const Eigen::Vector3d& end) {
        if (row < 0) {
            return; // 已取消选中
        }
        // start、end 与“起点”“终点”列一致，已计入内缩
    });

Eigen::Vector3d start;
Eigen::Vector3d end;
if (selectedWeldEndpoints(table, start, end)) {
    // 当前选中行
}

clearWeldList(table); // 或点击表上方的“清空”
```
