# Cursor

## Open3D GICP 点云配准

使用 Open3D 的 Generalized ICP（`registration_generalized_icp`）将源点云配准到目标点云。

```bash
pip install -r python/requirements.txt

# 配准两个点云
python python/gicp_registration.py source.pcd target.pcd --voxel-size 0.05 --output aligned.pcd

# 大位移时用由粗到精的多尺度 GICP
python python/gicp_registration.py source.pcd target.pcd --voxel-size 0.05 --multiscale

# 合成数据自检
python python/gicp_registration.py --self-test
python python/test_gicp_registration.py
```

```python
from gicp_registration import load_point_cloud, register_gicp

source = load_point_cloud("source.pcd")
target = load_point_cloud("target.pcd")
result = register_gicp(source, target, voxel_size=0.05)
source.transform(result.transformation)
print(result.fitness, result.inlier_rmse)
print(result.transformation)
```

GICP 是局部方法。初始位姿偏差较大时，先做全局粗配准（如 FPFH + RANSAC），再把结果作为 `init` 传入。

## ScaleAISShapeBy1000

将 OpenCASCADE 的 `AIS_Shape*` 缩小 1000 倍，并返回一个新的 `AIS_Shape*`。

```cpp
#include "ScaleAISShape.h"

AIS_Shape* ais = /* 已有对象 */;
AIS_Shape* scaled = ScaleAISShapeBy1000(ais);

// 推荐用 Handle 接管返回值，避免泄漏
Handle(AIS_Shape) scaledHandle = ScaleAISShapeBy1000(ais);
```
