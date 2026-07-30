#include "CylindricalHoleDetector.h"

#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>

namespace {

void printUsage(const char* argv0) {
  std::cerr
      << "Usage: " << argv0
      << " --step <file.step> --diameter <mm> [--tolerance <mm>] [--pcd <out.pcd>]\n"
      << "\n"
      << "  --step       Path to input STEP / STP file (required)\n"
      << "  --diameter   Target hole diameter in mm (required)\n"
      << "  --tolerance  Absolute diameter tolerance in mm (default: 0.05)\n"
      << "  --pcd        Optional path to write sampled PCL point cloud\n"
      << "  --deflection Linear mesh deflection in mm (default: 0.5)\n";
}

}  // namespace

int main(int argc, char** argv) {
  std::string step_path;
  std::string pcd_path;
  double diameter = -1.0;
  double tolerance = 0.05;
  double deflection = 0.5;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto need = [&](const char* name) -> std::string {
      if (i + 1 >= argc) {
        std::cerr << "Missing value for " << name << "\n";
        std::exit(1);
      }
      return argv[++i];
    };

    if (arg == "--step") {
      step_path = need("--step");
    } else if (arg == "--diameter") {
      diameter = std::stod(need("--diameter"));
    } else if (arg == "--tolerance") {
      tolerance = std::stod(need("--tolerance"));
    } else if (arg == "--pcd") {
      pcd_path = need("--pcd");
    } else if (arg == "--deflection") {
      deflection = std::stod(need("--deflection"));
    } else if (arg == "--help" || arg == "-h") {
      printUsage(argv[0]);
      return 0;
    } else {
      std::cerr << "Unknown argument: " << arg << "\n";
      printUsage(argv[0]);
      return 1;
    }
  }

  if (step_path.empty() || diameter <= 0.0) {
    printUsage(argv[0]);
    return 1;
  }

  CylindricalHoleDetector detector(step_path, diameter, tolerance);
  detector.setMeshDeflection(deflection);

  if (!detector.process()) {
    std::cerr << "Error: " << detector.lastError() << "\n";
    return 1;
  }

  const auto& cloud = detector.pointCloud();
  const auto& holes = detector.holes();

  std::cout << std::fixed << std::setprecision(4);
  std::cout << "File: " << step_path << "\n"
            << "Target diameter: " << diameter << " ± " << tolerance << " mm\n"
            << "Point cloud size: " << cloud->size() << "\n"
            << "Found " << holes.size() << " cylindrical hole(s):\n";

  for (std::size_t i = 0; i < holes.size(); ++i) {
    const HoleInfo& h = holes[i];
    std::cout << "  [" << (i + 1) << "] diameter=" << h.diameter_mm
              << " mm, depth≈" << h.depth_mm << " mm, faces=" << h.face_count
              << ",\n"
              << "       position=(" << h.position.X() << ", " << h.position.Y()
              << ", " << h.position.Z() << "),\n"
              << "       direction=(" << h.direction.X() << ", "
              << h.direction.Y() << ", " << h.direction.Z()
              << ")  [snapped to workpiece XYZ],\n"
              << "       raw_direction=(" << h.raw_direction.X() << ", "
              << h.raw_direction.Y() << ", " << h.raw_direction.Z() << ")\n";
  }

  if (!pcd_path.empty()) {
    if (!detector.savePointCloud(pcd_path)) {
      std::cerr << "Failed to write PCD: " << pcd_path << "\n";
      return 1;
    }
    std::cout << "Wrote point cloud: " << pcd_path << "\n";
  }

  return 0;
}
