# 在原有 ReadModel 上支持颜色渲染

基于你现有的 `readmodel.h` / `readmodel.cpp` 修改：保留 `readStlModel`、`readStepModel`、`writeStepModel`、`writeStlModel`，并新增 STEP 颜色读取与 `AIS_ColoredShape` 显示。

## 原有接口（保持不变）

- `readStlModel` — 仍用 `RWStl::ReadFile` 建 Face
- `readStepModel` — 仍用 `STEPControl_Reader`（仅几何）
- `writeStepModel` / `writeStlModel` — 逻辑未改

## 新增

- `readStepModelWithColors` — `STEPCAFControl_Reader` + XCAF，保留颜色
- `makeDisplayShape` — 生成带颜色的 `AIS_ColoredShape`

## 导入处改法

```cpp
#include "readmodel.h"

AIS_Shape* loadShape = nullptr;                 // 保持你原来的类型
Handle(AIS_ColoredShape) loadShapeHandle;       // 推荐：保住引用计数
const Quantity_Color tint(0.72, 0.74, 0.78, Quantity_TOC_RGB);

if (suffix == "stl") {
  loadShapeHandle = ReadModel::makeDisplayShape(
      ReadModel::readStlModel(filename.toStdString().c_str()), tint);
}
else if (suffix == "step" || suffix == "stp") {
  loadShapeHandle = ReadModel::makeDisplayShape(
      ReadModel::readStepModelWithColors(filename.toStdString().c_str()), tint);
}
else {
  ui->console->print(ct::LOG_INFO,
      QStringLiteral("导入文件格式非stl或step文件!"));
  return;
}

// Handle(AIS_ColoredShape) -> AIS_Shape*（子类指针隐式转基类）
loadShape = loadShapeHandle.get();

context->Display(loadShapeHandle, Standard_False);
context->SetDisplayMode(loadShapeHandle, AIS_Shaded, Standard_False);
context->UpdateCurrentViewer();
```

完整示例：`examples/load_with_color.cpp`。

## 链接库

需额外链接 `TKXCAF`、`TKXDESTEP`（见 `CMakeLists.txt`）。
