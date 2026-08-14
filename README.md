# 读取 STL / STEP 时记录单位

OpenCASCADE 的 `AIS_Shape` / `TopoDS_Shape` 本身没有单位。单位必须在读文件时记下。

| 格式 | 如何得到单位 |
|------|----------------|
| STEP | 解析 `LENGTH_UNIT`（毫米/米/厘米/英寸），否则看 XCAF；仍没有则记为 **mm** |
| STL | 标准无单位；仅当文件头出现 `mm` / `metre` 等字样时记录，否则 **unknown** |

```cpp
auto m = ReadModel::loadModel(path);          // .stl / .stp / .step，不缩放
auto m = ReadModel::loadStlModel(path);
auto m = ReadModel::loadStepModelWithColors(path);

m.sourceUnit;                                 // 文件原始单位
m.unit;                                       // 当前几何单位（load* 二者相同）
ReadModel::unitName(m.unit);                  // "mm" / "m" / "unknown" ...

Handle(AIS_ColoredShape) ais = ReadModel::makeDisplayShape(m);
ReadModel::unitOf(ais.get());                 // 从 AIS_Shape 取回当前单位
ReadModel::sourceUnitOf(ais.get());           // 文件原始单位

// 若要把毫米几何转到米：
if (m.unit == ReadModel::LengthUnit::Millimetre)
    m.shape = ReadModel::ScaleShape(m.shape, ReadModel::toMetres(m.unit));
```

旧接口 `readStepModel` / `readStepModelWithColors` 仍会把毫米缩放到米：返回的 `shape` 与 `unit` 为米，`sourceUnit` 仍是文件单位。
