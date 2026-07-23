# Weld Coordinate System (OpenCASCADE)

Compute a weld local frame and a discretized seam trajectory from a selected edge.

## Trajectory API

```cpp
#include "WeldCoordinateSystem.hxx"

struct DiscretePoint {
  gp_Pnt position; // point on edge
  gp_Dir xDir;     // Y × Z (right-hand)
  gp_Dir yDir;     // edge tangent / travel
  gp_Dir zDir;     // bisector of the two adjacent face normals
};

std::vector<DiscretePoint> trajectory;
DiscretizeWeldTrajectory(selectShape, edge, trajectory, /*spacingMm=*/10.0);
```

### Frame at each sample

| Axis | Definition |
|------|------------|
| **Y** | Edge tangent (edge orientation) |
| **Z** | Unit bisector of the two adjacent face normals, in the plane ⊥ Y |
| **X** | `Y × Z` (right-handed: `X × Y = Z`) |

Samples are spaced by **10 mm** arc length along the edge (endpoints always included). The edge must be shared by exactly two faces. OpenCASCADE **7.4+**.

## Build

```bash
cmake -S . -B build -DOpenCASCADE_DIR=/path/to/opencascade/lib/cmake/opencascade
cmake --build build
```
