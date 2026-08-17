# STEP 读取单位：mm 绕圆心缩放到米，m 原样读取

在原有 `readmodel.h` / `readmodel.cpp` 上做最小改动，**接口不变**。

- **mm**：`ScaleShape(shape, 0.001)`，缩放中心为包围盒圆心 `(x, y, z)`
- **m**：直接读取，不转换

`readStlModel` / `readStepModel` / `readStepModelWithColors` / `makeDisplayShape` / `writeStepModel` / `writeStlModel` 签名均未改。

新增：

```cpp
loadShape = ReadModel::ScaleAis(loadColorShape.get(), 0.001);
```

`ScaleAis` 输入 `AIS_Shape*`，绕包围盒圆心缩小后返回**新的** `AIS_Shape*`（不修改、不返回原指针）。建议用 Handle 接住以免对象无主：

```cpp
Handle(AIS_Shape) scaledHandle = ReadModel::ScaleAis(loadColorShape.get(), 0.001);
loadShape = scaledHandle.get();
```
