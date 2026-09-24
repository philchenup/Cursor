# Cursor

## 点云 Surface Matching

在「只使用点云、不训练、结果可公开核对」的约束下，效果最好的 surface matching
算法是 **改进 PPF（Vidal-Sensors18）+ 点到面 ICP**。

依据、和相邻任务（配准 / 重建）的对照见 [docs/surface_matching.md](docs/surface_matching.md)。
本仓库用纯 NumPy 实现了同一条管线，可在合成 L 型支架上复现位姿恢复：

```bash
python -m pip install -r requirements.txt
python -m surface_matching.demo
python -m pytest -q
```

核心入口是 `surface_matching.match_surface(model_points, scene_points, model_normals, scene_normals)`。
工业部署更建议直接用 OpenCV `ppf_match_3d`（官方维护，同一算法家族）。

## ScaleAISShapeBy1000

将 OpenCASCADE 的 `AIS_Shape*` 缩小 1000 倍，并返回一个新的 `AIS_Shape*`。

```cpp
#include "ScaleAISShape.h"

AIS_Shape* ais = /* 已有对象 */;
AIS_Shape* scaled = ScaleAISShapeBy1000(ais);

// 推荐用 Handle 接管返回值，避免泄漏
Handle(AIS_Shape) scaledHandle = ScaleAISShapeBy1000(ais);
```
