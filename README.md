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
- **Qhull** link library (`qhull_r` / `qhullstatic_r`) — required at link time
- Components: `common`, `surface`, `io` (io only for the example)

## Build

```bash
cmake -S . -B build
cmake --build build -j
```

## Windows / MSVC: `LNK2001 qh_*`

If Visual Studio reports unresolved externals such as:

```text
error LNK2001: unresolved external symbol qh_new_qhull
error LNK2001: unresolved external symbol qh_freeqhull
error LNK2001: unresolved external symbol qh_lib_check
...
```

`pcl::ConvexHull` only *declares* these Qhull APIs; your final executable/DLL must **link the Qhull library**.

### Fix (Visual Studio project, e.g. MakeTool)

1. Install Qhull (same flavor your PCL was built against), e.g. via vcpkg:
   ```text
   vcpkg install qhull:x64-windows
   ```
2. Project Properties → **Linker** → **General** → **Additional Library Directories**  
   add the folder that contains `qhull_r.lib` or `qhullstatic_r.lib`
   (often `...\vcpkg\installed\x64-windows\lib` or your PCL `lib` dir).
3. Project Properties → **Linker** → **Input** → **Additional Dependencies**  
   add one of:
   - `qhull_r.lib` (shared / import lib; common with dynamic PCL)
   - `qhullstatic_r.lib` (static; try this if `qhull_r.lib` still fails)
4. Also keep linking PCL surface deps, e.g. `pcl_surface.lib`, `pcl_common.lib`, …
5. Prefer CMake/`find_package(PCL)` so Qhull is pulled transitively:
   ```cmake
   find_package(PCL 1.12 REQUIRED COMPONENTS common surface)
   target_link_libraries(MakeTool PRIVATE ${PCL_LIBRARIES})
   # if still missing qh_*:
   target_link_libraries(MakeTool PRIVATE QHULL::QHULL) # or qhull_r.lib
   ```

### Notes

- Symbol names `qh_*` come from **Qhull**, not from this header itself.
- Match **x64/x86** and **Debug/Release** of Qhull to your MakeTool configuration.
- If PCL was built against static Qhull, prefer `qhullstatic_r.lib`.

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
