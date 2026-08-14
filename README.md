# STEP 读取：毫米模型绕原点缩小 1000 倍

OpenCASCADE 场景坐标无单位。毫米 STEP 读入后绕 **原点** 缩小 1000 倍即转为米；米制文件原样读取。

```cpp
TopoDS_Shape shape = ReadModel::readStepModel(path);          // 内部已按单位处理
shape = ReadModel::ScaleShape(shape);                         // 也可手动：绕原点 ×0.001
shape = ReadModel::ScaleShape(shape, 0.001);                  // 显式因子
```

`ScaleShape` 使用 `gp_Trsf::SetScale(gp_Pnt(0,0,0), factor)`，几何相对原点的位置一并换算（1 mm → 0.001 m）。

已显示的 `AIS_Shape*` 用局部变换缩放，不改 BRep：

```cpp
ReadModel::ScaleAisShape(ais);           // 绕原点 ×0.001
ReadModel::ScaleAisShape(ais, 0.001);
```
