# AIS_Shape 加载时渲染颜色

`AIS_Shape(scene_shape)` **不会**自动带上 STEP 颜色：`STEPControl_Reader` / `TopoDS_Shape` 只含几何。颜色在 XCAF 文档里，需要 `STEPCAFControl_Reader` + `AIS_ColoredShape`。

## 原因

| 方式 | 结果 |
|------|------|
| `readStepModel` → `new AIS_Shape(shape)` | 只有几何，默认灰/材质色 |
| `readStepModelWithColors` → `AIS_ColoredShape` + XCAF 颜色 | 可渲染 STEP 面/实体色 |
| STL | 文件本身无 CAD 颜色，只能设单一显示色 |

## 把原来的导入代码换成

```cpp
#include "ReadModel.h"
#include <AIS_ColoredShape.hxx>

// 建议成员改为：
// Handle(AIS_ColoredShape) loadShape;

const Quantity_Color tint(0.72, 0.74, 0.78, Quantity_TOC_RGB);

if (suffix == "stl") {
  TopoDS_Shape scene_shape =
      ReadModel::readStlModel(filename.toStdString().c_str());
  loadShape = ReadModel::makeDisplayShape(scene_shape, tint);
}
else if (suffix == "step" || suffix == "stp") {
  auto colored =
      ReadModel::readStepModelWithColors(filename.toStdString().c_str());
  loadShape = ReadModel::makeDisplayShape(colored, tint);
}
else {
  ui->console->print(ct::LOG_INFO,
      QStringLiteral("导入文件格式非stl或step文件!"));
  return;
}

context->Display(loadShape, Standard_False);
context->SetDisplayMode(loadShape, AIS_Shaded, Standard_False);
context->UpdateCurrentViewer();
```

完整示例见 `examples/load_with_color.cpp`。

## 若暂时必须保留 `AIS_Shape*`（仅单色）

```cpp
loadShape = new AIS_Shape(scene_shape);
loadShape->SetColor(Quantity_Color(0.72, 0.74, 0.78, Quantity_TOC_RGB));
loadShape->SetDisplayMode(AIS_Shaded);
loadShape->Attributes()->SetFaceBoundaryDraw(Standard_True);
```

这只能设**一种**颜色，无法显示 STEP 多色。

## CMake 需链接的 OCCT 库

```
TKXCAF TKXDESTEP TKCAF TKLCAF TKV3d TKSTEP TKSTL ...
```

见根目录 `CMakeLists.txt`。

## API

- `ReadModel::readStlModel` — STL 几何  
- `ReadModel::readStepModel` — STEP 几何（无色）  
- `ReadModel::readStepModelWithColors` — STEP + XCAF 颜色文档  
- `ReadModel::makeDisplayShape` — 生成可着色的 `AIS_ColoredShape`
