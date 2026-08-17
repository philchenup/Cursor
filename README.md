# Cursor

## GetWorldTransformation / GetWorldPosition

获取 `TopoDS_Shape` 或 `AIS_Shape` 在世界坐标系中的位姿与位置。

- `TopoDS_Shape`：来自 `Location()`（几何自身坐标系）。
- `AIS_Shape`：父节点链 × `LocalTransformation()`（显示坐标系）。不要绑定 `ais->Transformation()` 的引用。

```cpp
#include "ShapeWorldLocation.h"

Handle(AIS_Shape) ais = /* 已有对象 */;

gp_Trsf worldTrsf = GetWorldTransformation(ais);  // 世界位姿
gp_Pnt origin = GetWorldPosition(ais);            // 局部原点的世界坐标
gp_Pnt center = GetWorldCenter(ais);              // 包围盒中心的世界坐标

const TopoDS_Shape& shape = ais->Shape();
gp_Pnt shapeOrigin = GetWorldPosition(shape);     // Location 原点
```

## ScaleAISShapeBy1000

将 OpenCASCADE 的 `AIS_Shape*` 缩小 1000 倍，并返回一个新的 `AIS_Shape*`。

```cpp
#include "ScaleAISShape.h"

AIS_Shape* ais = /* 已有对象 */;
AIS_Shape* scaled = ScaleAISShapeBy1000(ais);

// 推荐用 Handle 接管返回值，避免泄漏
Handle(AIS_Shape) scaledHandle = ScaleAISShapeBy1000(ais);
```
