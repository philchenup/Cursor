# Cursor

## RGB-D 点云恢复（Python + Open3D）

读取一张 RGB PNG 和一张 **16 位**深度 PNG，按针孔内参反投影为彩色点云，并用 Open3D 写出 `.ply` / `.pcd`。

针孔模型（相机系：x 右、y 下、z 前）：

```
X = (u - cx) * Z / fx
Y = (v - cy) * Z / fy
Z = depth / depth_scale
```

16 位深度图通常以毫米存储，此时 `--depth-scale 1000`，点云单位为米。

```bash
pip install -r python/requirements.txt

# 真实数据
python python/rgbd_to_pointcloud.py \
  --rgb color.png \
  --depth depth16.png \
  --fx 525 --fy 525 --cx 319.5 --cy 239.5 \
  --depth-scale 1000 \
  --output cloud.ply

# 或使用内参 JSON（见 python/intrinsics.example.json）
python python/rgbd_to_pointcloud.py \
  --rgb color.png --depth depth16.png \
  --intrinsics python/intrinsics.example.json \
  -o cloud.ply

# 无真实图时，合成 RGB + 16 位平面深度并跑通全流程
python python/rgbd_to_pointcloud.py --demo -o python/demo_output/cloud.ply
```

库接口：`load_rgb_png`、`load_depth_png16`、`reconstruct_point_cloud`、`save_point_cloud`。

```bash
python python/test_rgbd_to_pointcloud.py
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
