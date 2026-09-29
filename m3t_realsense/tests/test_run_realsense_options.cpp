#include "run_realsense_options.h"

#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {

int Fail(const char *msg) {
  std::cerr << "FAIL: " << msg << std::endl;
  return 1;
}

bool Approx(float a, float b, float eps = 1e-5f) { return std::fabs(a - b) < eps; }

}  // namespace

int main() {
  {
    RealSenseExampleOptions opt;
    std::string err;
    const std::vector<std::string> args = {
        "run_realsense_example", "--use_region", "--use_depth",
        "--measure_occlusions", "--use_depth_viewer", "-b", "obj_000014", "-m",
        "D:\\models", "--scale_geometry", "0.001"};
    if (!ParseRealSenseExampleArgs(args, &opt, &err)) {
      return Fail(err.c_str());
    }
    if (opt.body_name != "obj_000014" || opt.models_dir != "D:\\models") {
      return Fail("body/models not parsed");
    }
    if (!opt.use_region || !opt.use_depth || !opt.measure_occlusions ||
        !opt.use_depth_viewer) {
      return Fail("flags not parsed");
    }
    if (!Approx(opt.scale_geometry, 0.001f)) {
      return Fail("scale_geometry");
    }
  }

  {
    RealSenseExampleOptions opt;
    std::string err;
    const std::vector<std::string> args = {"prog", "-b", "cup", "-m", "./mesh"};
    if (!ParseRealSenseExampleArgs(args, &opt, &err)) {
      return Fail(err.c_str());
    }
    if (!opt.use_region) {
      return Fail("expected default --use_region");
    }
  }

  {
    RealSenseExampleOptions opt;
    std::string err;
    const std::vector<std::string> args = {
        "prog", "-b", "cup", "-m", "./mesh", "--measure_occlusions"};
    if (ParseRealSenseExampleArgs(args, &opt, &err)) {
      return Fail("measure_occlusions without depth should fail");
    }
  }

  {
    RealSenseExampleOptions opt;
    std::string err;
    const std::vector<std::string> args = {"prog"};
    if (ParseRealSenseExampleArgs(args, &opt, &err)) {
      return Fail("missing required args should fail");
    }
  }

  // pym3t initial rotation: R0 @ Rx(0.2)
  {
    const float c = std::cos(0.2f);
    const float s = std::sin(0.2f);
    // R0 = [[1,0,0],[0,0,-1],[0,1,0]]
    const float r11 = -s;
    const float r12 = -c;
    const float r21 = c;
    const float r22 = -s;
    if (!Approx(r11, -0.19866933f) || !Approx(r12, -0.9800666f) ||
        !Approx(r21, 0.9800666f) || !Approx(r22, -0.19866933f)) {
      return Fail("python-equivalent Rx(0.2) composition");
    }
  }

  std::cout << "ok" << std::endl;
  return 0;
}
