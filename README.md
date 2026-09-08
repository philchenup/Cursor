# Cursor

## fitCircle3D

使用 PCL 以最小二乘拟合三维空间圆（对应 Python `opt.leastsq` + `spherrors`）：PCA 投影到平面后，用残差 `(x-a)^2+(y-b)^2-r^2` 求圆心与半径。

```cpp
#include "FitCircle3D.h"

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = /* 点云 */;
cv::Point3f center;
float radius = 0.f;
if (fitCircle3D(cloud, center, radius)) {
    // center, radius
}
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
