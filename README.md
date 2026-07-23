# Weld Coordinate System (OpenCASCADE)

Compute a weld local frame and a discretized seam trajectory from a selected edge.

## Trajectory API

```cpp
#include "WeldCoordinateSystem.hxx"

WeldDiscretizeOptions opt;
opt.spacingMm = 10.0;          // 焊缝插值间距
opt.reverseZ  = Standard_False; // true = 二分角反向
opt.retractMm = 50.0;          // 起弧/收弧沿 -Z 后退距离 ← 在这里改

std::vector<DiscretePoint> trajectory;
DiscretizeWeldTrajectory(selectShape, edge, trajectory, opt);
```

Defaults: `kDefaultSeamSpacingMm = 10`, `kDefaultArcRetractMm = 50`.

### Frame at each sample

| Axis | Definition |
|------|------------|
| **Y** | Edge tangent (edge orientation) |
| **Z** | Unit bisector of the two adjacent face normals; `opt.reverseZ = true` flips |
| **X** | `Y × Z` (right-handed: `X × Y = Z`) |

### Arc start / end points

When `opt.retractMm > 0`:

| Point | Index | Position |
|-------|-------|----------|
| 起弧点 | `trajectory.front()` | first seam − `retractMm * zDir` |
| 焊缝点 | middle | samples every `spacingMm` |
| 收弧点 | `trajectory.back()` | last seam − `retractMm * zDir` |

Set `opt.retractMm = 0` to disable. OpenCASCADE **7.4+**.

## Build

```bash
cmake -S . -B build -DOpenCASCADE_DIR=/path/to/opencascade/lib/cmake/opencascade
cmake --build build
```
