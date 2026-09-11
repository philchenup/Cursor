# Cursor

## Hand-eye calibration (Eigen)

点法手眼标定，仅依赖 Eigen。

**眼在手上**（相机装在法兰上）需要相机点、法兰位姿、TCP：

```cpp
auto r = CalibrateEyeInHand(points_in_camera, flanges_in_base, tcps_in_base);
// r.T_flange_camera == ^{F}T_{C}
// ^{B}P = ^{B}T_{F} · ^{F}T_{C} · ^{C}P
```

**眼在手外**（相机固定）只需相机点和 TCP，不要法兰位姿：

```cpp
auto r = CalibrateEyeOnHand(points_in_camera, tcps_in_base);
// r.T_base_camera == ^{B}T_{C}
// ^{B}P = ^{B}T_{C} · ^{C}P
```

两组都是 Kabsch 刚体配准，\(N \ge 3\) 且不共线。

```bash
cmake -S . -B build -DCMAKE_CXX_COMPILER=g++ && cmake --build build
./build/hand_eye_calibration_example
```

## ScaleAISShapeBy1000

将 OpenCASCADE 的 `AIS_Shape*` 缩小 1000 倍，并返回一个新的 `AIS_Shape*`。

```cpp
#include "ScaleAISShape.h"

AIS_Shape* ais = /* 已有对象 */;
AIS_Shape* scaled = ScaleAISShapeBy1000(ais);

Handle(AIS_Shape) scaledHandle = ScaleAISShapeBy1000(ais);
```
