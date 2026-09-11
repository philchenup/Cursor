# Cursor

## CalibrateEyeInHand

眼在手上（相机装在法兰上）的点法手眼标定。每组数据需要：标定点在相机系的 XYZ、拍照时法兰位姿 `^{B}T_{F}`、同一点在基座下的 TCP XYZ；不少于 3 组且对应点不共线。

```cpp
#include "HandEyeCalibration.h"

EyeInHandCalibResult result = CalibrateEyeInHand(
    points_in_camera,   // ^{C}P_i
    flanges_in_base,    // ^{B}T_{F,i}
    tcps_in_base);      // ^{B}P_i  （TCP 触碰得到）

// result.T_flange_camera 即 ^{F}T_{C}
```

数学流程：`^{F}P_i = ^{B}T_{F,i}^{-1} {}^{B}P_i`，再对 `(^{C}P_i, ^{F}P_i)` 做 Kabsch–Umeyama / PCL SVD，得到刚体变换 `X = ^{F}T_{C}`。详见 `include/HandEyeCalibration.h`。

示例读取 `data/` 下三份 TXT（相机点 XYZ、法兰 `x,y,z,rx,ry,rz` 度、TCP XYZ），计算并打印 \(^{F}T_{C}\)：

```bash
cmake -S . -B build -DCMAKE_CXX_COMPILER=g++ && cmake --build build
./build/hand_eye_calibration_example
# 或指定路径：
# ./build/hand_eye_calibration_example camera_point.txt robot_flange_pose.txt tcp_pose.txt
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
