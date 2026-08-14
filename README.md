# STEP 读取时按单位换算到米

用 OpenCASCADE 读取 STEP 时判断长度单位：

- **mm**：以包围盒圆心 `(x, y, z)` 为缩放中心，按 `0.001` 缩放到 **m**，同时把圆心坐标也换算到米（世界坐标一致）。
- **m**：直接读取，不转换。

`writeStepModel` 仍以米写出（`write.step.unit = M`）。

## 接口

```cpp
TopoDS_Shape shape = ReadModel::readStepModel(path);

ReadModel::ColoredModel colored = ReadModel::readStepModelWithColors(path);
// colored.sourceUnit          文件原始单位
// colored.convertedToMetres   mm（或其它非米单位）时为 true
```

`ScaleShape(shape, factor)` 改为绕包围盒圆心缩放（不再绕原点）。

## 行为说明

1. `ReadFile` 之后从 STEP 的 `GLOBAL_UNIT_ASSIGNED_CONTEXT` / `SI_UNIT` / `CONVERSION_BASED_UNIT` 判断长度单位。
2. 将 `read.step.unit` 设为文件自身单位再 `Transfer`，避免 OCCT 把米先换成毫米。
3. 仅当单位不是米时，绕圆心缩放并换算到米。

## 构建

需要已安装 OpenCASCADE 开发包。

```bash
cmake -S . -B build
cmake --build build
./build/read_step_units model.step
```
