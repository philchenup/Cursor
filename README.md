# Cylindrical Hole Detector (OCCT ≥ 7.4 + PCL)

Detect cylindrical holes in a STEP model with Open CASCADE Technology, sample the
surface into a PCL point cloud, and report each hole center plus an axis snapped
to the workpiece principal directions (±X / ±Y / ±Z).

## Class API

```cpp
#include "CylindricalHoleDetector.h"

CylindricalHoleDetector detector("part.step", /*diameter_mm=*/8.0, /*tolerance_mm=*/0.05);
detector.setMeshDeflection(0.5);   // denser cloud → smaller value

if (!detector.process()) {
  std::cerr << detector.lastError() << std::endl;
  return 1;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = detector.pointCloud();
const std::vector<HoleInfo>& holes = detector.holes();

for (const HoleInfo& h : holes) {
  // h.position   — hole axis reference point (mm)
  // h.direction  — snapped to nearest workpiece ±X/±Y/±Z
  // h.raw_direction — geometric cylinder axis before snapping
  // h.diameter_mm / h.depth_mm / h.face_count
}

detector.savePointCloud("part.pcd");
```

### Inputs

| Parameter | Meaning | Default |
|-----------|---------|---------|
| STEP path | `.step` / `.stp` model | required |
| diameter  | target hole diameter (mm) | required |
| tolerance | absolute diameter tolerance (mm) | `0.05` |

### Outputs

- **Intermediate STL** — B-Rep meshed and written via `StlAPI_Writer` (default: beside the STEP file)
- **PCL point cloud** — CAD2PCD area-weighted sampling over the STL triangle mesh (`pcl::PointXYZ`)
- **Hole list** — diameter, position, direction (workpiece XYZ), depth estimate, face count

## Algorithm

1. Read STEP → B-Rep shape / solid  
2. Keep cylindrical faces whose diameter matches target ± tolerance  
3. Classify hole vs boss: a point on the cylinder axis that lies **outside** the solid is a cavity  
4. Cluster coaxial faces of similar radius into one logical hole  
5. Snap each axis to the nearest ±X / ±Y / ±Z  
6. Mesh → export **STL** → `pcl::io::loadPolygonFileSTL` → area-weighted surface sample → PCL  

## Build

Requires:

- CMake ≥ 3.16  
- C++17 compiler  
- OpenCASCADE / OCCT ≥ 7.4 (`TKSTL` or equivalent for STL export)  
- PCL ≥ 1.10 (`common`, `io`, with VTK support for `loadPolygonFileSTL`)

```bash
mkdir build && cd build
cmake .. -DOpenCASCADE_DIR=/path/to/occt/lib/cmake/opencascade \
         -DPCL_DIR=/path/to/pcl/share/pcl-1.x
cmake --build . -j
```

## CLI example

```bash
./detect_holes --step part.step --diameter 8 --tolerance 0.05 \
               --samples 50000 --stl part.stl --pcd part.pcd
```

```cpp
detector.setSampleCount(50000);
detector.setStlOutputPath("part.stl");
```

## Layout

```
include/CylindricalHoleDetector.h   # public API
src/CylindricalHoleDetector.cpp     # OCCT + PCL implementation
examples/main.cpp                   # command-line demo
CMakeLists.txt
```
