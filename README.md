# Cursor

## Offline weld seam from a PLY point cloud

ROS-free port of [romi-lab/robotic-welding-demo](https://github.com/romi-lab/robotic-welding-demo).
The original `demo_all.py` subscribed to a RealSense topic, converted ROS `PointCloud2`
to Open3D, then drove a UR arm. This package keeps the same geometry pipeline and
drops ROS / robot I/O:

1. voxel downsample and outlier removal
2. normal estimation
3. **asymmetry** feature → keep the top 5%
4. DBSCAN, take the largest groove cluster
5. PCA thinning + directional sort + B-spline
6. 6-DoF torch poses, plus the original left/right multilayer offsets

Input is a `.ply` point cloud (already in the working frame). Output is trajectory
coordinates (`x,y,z` + rotation vector + quaternion) and a figure.

```bash
python3 -m pip install -r weld_seam_offline/requirements.txt
python3 -m weld_seam_offline --make-sample --out-dir output/weld_seam
python3 -m weld_seam_offline path/to/workpiece.ply --out-dir output/weld_seam
```

Generated files:

- `trajectory.csv` — waypoints `x,y,z,rx,ry,rz,qx,qy,qz,qw`
- `trajectory_left.csv` / `trajectory_right.csv` — V-groove fill passes
- `trajectory.png` — 3D / top / side views and a waypoint table
- `summary.json`, `weld_seam.npz`

Optional flags: `--voxel-size`, `--keep-ratio`, `--max-depth` (camera-frame Z cut), `--tcp-offset`.

```bash
python3 -m unittest tests.test_weld_seam_offline
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
