# Weld Coordinate System (OpenCASCADE)

Compute a right-handed local frame for a weld seam from a selected edge.

## API

```cpp
#include "WeldCoordinateSystem.hxx"

Standard_Boolean ComputeWeldCoordinateSystem(
    const TopoDS_Shape& selectShape,  // owner solid / shell
    const TopoDS_Edge&   edge,         // selected weld edge (+Y)
    gp_Ax3&             weldAxis);    // output frame
```

## Axis convention

| Axis | Definition |
|------|------------|
| **Origin** | Mid-point of the edge |
| **Y** | Edge tangent in the edge's topological orientation |
| **Z** | Unit bisector of the two adjacent face normals (open / exterior dihedral side), projected orthogonal to Y |
| **X** | Right-hand rule: `X = Y × Z` |

Requires the edge to be shared by **exactly two faces**. OpenCASCADE **7.4+**.

## Build

```bash
cmake -S . -B build -DOpenCASCADE_DIR=/path/to/opencascade/lib/cmake/opencascade
cmake --build build
```
