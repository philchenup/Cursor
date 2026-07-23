# Weld Coordinate System (OpenCASCADE)

Compute a weld local frame and a discretized seam trajectory from a selected edge.

## Trajectory API

```cpp
#include "WeldCoordinateSystem.hxx"

struct DiscretePoint {
  gp_Pnt position; // point on edge (or retracted arc point)
  gp_Dir xDir;     // Y × Z (right-hand)
  gp_Dir yDir;     // edge tangent / travel
  gp_Dir zDir;     // bisector of the two adjacent face normals
};

std::vector<DiscretePoint> trajectory;

// spacing=10, reverseZ=false, retract=50 → 起弧/收弧沿 -Z 后退 50
DiscretizeWeldTrajectory(selectShape, edge, trajectory, 10.0, Standard_False, 50.0);

// Z along the opposite bisector (二分角反向)
DiscretizeWeldTrajectory(selectShape, edge, trajectory, 10.0, Standard_True, 50.0);
```

### Frame at each sample

| Axis | Definition |
|------|------------|
| **Y** | Edge tangent (edge orientation) |
| **Z** | Unit bisector of the two adjacent face normals, in the plane ⊥ Y; pass `reverseZ=Standard_True` to flip |
| **X** | `Y × Z` (right-handed: `X × Y = Z`) |

### Arc start / end points

When `retractMm > 0` (default **50**):

| Point | Index | Position |
|-------|-------|----------|
| 起弧点 | `trajectory.front()` | first seam point − `retractMm * zDir` |
| 焊缝点 | middle | 10 mm arc-length samples on the edge |
| 收弧点 | `trajectory.back()` | last seam point − `retractMm * zDir` |

Pass `retractMm = 0` to disable. OpenCASCADE **7.4+**.

## Build

```bash
cmake -S . -B build -DOpenCASCADE_DIR=/path/to/opencascade/lib/cmake/opencascade
cmake --build build
```
