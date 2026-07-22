# Weld Coordinate System (OpenCASCADE)

Compute a weld local frame and a discretized seam trajectory from a selected edge.

## Trajectory API

```cpp
#include "WeldCoordinateSystem.hxx"

struct DiscretePoint {
  gp_Pnt position; // point on edge
  gp_Dir xDir;     // tangent / travel
  gp_Dir yDir;     // zDir × xDir
  gp_Dir zDir;     // 45° down vs world XOY, ⊥ xDir
};

std::vector<DiscretePoint> trajectory;
DiscretizeWeldTrajectory(selectShape, edge, trajectory, /*spacingMm=*/10.0);
```

### Frame at each sample

| Axis | Definition |
|------|------------|
| **X** | Edge tangent (edge orientation) |
| **Z** | Perpendicular to X; angle with world **XOY** plane = 45°, pointing down. If two solutions exist, pick the one closer to the adjacent-face normal bisector |
| **Y** | `Z × X` (right-handed: `X × Y = Z`) |

Samples are spaced by **10 mm** arc length along the edge (endpoints always included). The edge must be shared by exactly two faces. OpenCASCADE **7.4+**.

## Build

```bash
cmake -S . -B build -DOpenCASCADE_DIR=/path/to/opencascade/lib/cmake/opencascade
cmake --build build
```
