# PCL Hidden Point Removal (Open3D port)

PCL (>= 1.12) C++ port of Open3D's `geometry::PointCloud::HiddenPointRemoval`
(Katz et al., *Direct Visibility of Point Sets*, 2007).

## Algorithm (same as Open3D)

1. Spherical-flip each point relative to `camera_location` with radius `R`
2. Append the projection-space origin
3. Compute the 3D convex hull (`pcl::ConvexHull` / Qhull)
4. Map hull vertices back to the original cloud and drop the origin vertex
5. Return the visible triangle mesh and original-cloud indices

## Requirements

- C++14
- PCL **1.12+** built with **Qhull** (`pcl::ConvexHull`)
- Components: `common`, `surface`, `io` (io only for the example)

## Build

```bash
cmake -S . -B build
cmake --build build -j
```

## Usage

```cpp
#include "hidden_point_removal.h"

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
// ... fill cloud ...

Eigen::Vector3d camera(0.0, 0.0, 2.0);
double radius = 100.0;  // typically ~100x scene diameter (Open3D tutorial)

auto result = pcl_utils::HiddenPointRemoval<pcl::PointXYZ>(cloud, camera, radius);
auto visible = pcl_utils::ExtractVisiblePoints<pcl::PointXYZ>(
    cloud, result.visible_indices);

// result.mesh              -> pcl::PolygonMesh of visible surface
// result.visible_indices   -> indices into the original cloud
```

### Example binary

```bash
./build/hidden_point_removal_example                 # synthetic cloud
./build/hidden_point_removal_example in.pcd out.pcd  # PCD in/out
```

## Source mapping

| Open3D | This port |
| --- | --- |
| `cpp/open3d/geometry/PointCloud.cpp` `HiddenPointRemoval` | `include/hidden_point_removal.h` |
| `Qhull::ComputeConvexHull` | `pcl::ConvexHull` |
| `TriangleMesh` + `pt_map` | `pcl::PolygonMesh` + `visible_indices` |
