// SPDX-License-Identifier: MIT
//
// C++ equivalent of pym3t examples/run_realsense_example.py
// (https://github.com/agimus-project/pym3t) using the M3T library from
// https://github.com/DLR-RM/3DObjectTracking (agimus-project fork).
//
// Color-only:
//   run_realsense_example.exe --use_region -b obj_000014 -m D:\models
// Color + depth:
//   run_realsense_example.exe --use_region --use_depth --measure_occlusions -b obj_000014 -m D:\models
//
// Viewer keys (same as pym3t / M3T Tracker):
//   Q  quit
//   D  run detection (apply the static initial pose)
//   X  detect and start tracking
//   T  start tracking
//   S  stop tracking

#include "run_realsense_options.h"

#include <m3t/basic_depth_renderer.h>
#include <m3t/body.h>
#include <m3t/common.h>
#include <m3t/depth_modality.h>
#include <m3t/depth_model.h>
#include <m3t/link.h>
#include <m3t/normal_viewer.h>
#include <m3t/optimizer.h>
#include <m3t/publisher.h>
#include <m3t/realsense_camera.h>
#include <m3t/region_modality.h>
#include <m3t/region_model.h>
#include <m3t/renderer_geometry.h>
#include <m3t/silhouette_renderer.h>
#include <m3t/static_detector.h>
#include <m3t/texture_modality.h>
#include <m3t/tracker.h>

#include <Eigen/Geometry>

#include <filesystem>
#include <iomanip>
#include <iostream>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <shellapi.h>
#endif

namespace {

#ifdef _WIN32
std::vector<std::string> Utf8Args() {
  int wargc = 0;
  LPWSTR *wargv = CommandLineToArgvW(GetCommandLineW(), &wargc);
  std::vector<std::string> args;
  args.reserve(static_cast<size_t>(wargc));
  for (int i = 0; i < wargc; ++i) {
    const int n = WideCharToMultiByte(CP_UTF8, 0, wargv[i], -1, nullptr, 0,
                                      nullptr, nullptr);
    std::string s(static_cast<size_t>(n - 1), '\0');
    WideCharToMultiByte(CP_UTF8, 0, wargv[i], -1, s.data(), n, nullptr,
                        nullptr);
    args.push_back(std::move(s));
  }
  LocalFree(wargv);
  return args;
}

std::filesystem::path PathFromUtf8(const std::string &utf8) {
  return std::filesystem::u8path(utf8);
}
#else
std::vector<std::string> Utf8Args(int argc, char **argv) {
  return std::vector<std::string>(argv, argv + argc);
}

std::filesystem::path PathFromUtf8(const std::string &utf8) {
  return std::filesystem::path(utf8);
}
#endif

// Matches pym3t run_realsense_example.py:
//   body2world = [[1,0,0,0],[0,0,-1,0],[0,1,0,0.556],[0,0,0,1]]
//   then R = R @ Rx(0.2)
m3t::Transform3fA MakeDefaultBody2WorldPose() {
  Eigen::Matrix4f matrix;
  matrix << 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f, 0.0f, 0.0f, 1.0f, 0.0f,
      0.556f, 0.0f, 0.0f, 0.0f, 1.0f;
  m3t::Transform3fA pose;
  pose.matrix() = matrix;
  const Eigen::Matrix3f dR =
      Eigen::AngleAxisf(0.2f, Eigen::Vector3f::UnitX()).toRotationMatrix();
  pose.linear() = pose.linear() * dR;
  return pose;
}

class PosePrinter : public m3t::Publisher {
 public:
  PosePrinter(const std::string &name, std::shared_ptr<m3t::Body> body)
      : Publisher(name), body_(std::move(body)) {}

  bool SetUp() override {
    set_up_ = body_ != nullptr;
    return set_up_;
  }

  bool UpdatePublisher(int iteration) override {
    if (!set_up_ || !body_) return false;
    const Eigen::Matrix4f m = body_->body2world_pose().matrix();
    std::ostringstream oss;
    oss << std::fixed;
    oss << "iter " << iteration << " body2world =\n";
    for (int r = 0; r < 4; ++r) {
      oss << "  ";
      for (int c = 0; c < 4; ++c) {
        oss << m(r, c);
        if (c < 3) oss << ' ';
      }
      oss << '\n';
    }
    std::cout << oss.str();
    return true;
  }

 private:
  std::shared_ptr<m3t::Body> body_;
};

}  // namespace

int main(int argc, char **argv) {
#ifdef _WIN32
  (void)argc;
  (void)argv;
  const std::vector<std::string> args = Utf8Args();
#else
  const std::vector<std::string> args = Utf8Args(argc, argv);
#endif

  RealSenseExampleOptions opt;
  std::string parse_error;
  if (!ParseRealSenseExampleArgs(args, &opt, &parse_error)) {
    if (parse_error != "help") {
      std::cerr << parse_error << std::endl;
    }
    PrintRealSenseExampleUsage(
        args.empty() ? "run_realsense_example" : args[0].c_str());
    return parse_error == "help" ? 0 : -1;
  }

  const std::filesystem::path models_dir = PathFromUtf8(opt.models_dir);
  const std::filesystem::path tmp_dir = PathFromUtf8(opt.tmp_dir);
  const std::filesystem::path obj_path =
      models_dir / (opt.body_name + ".obj");
  if (!std::filesystem::exists(obj_path)) {
    std::cerr << "Model not found: " << obj_path.u8string() << std::endl;
    return -1;
  }
  std::filesystem::create_directories(tmp_dir);

  // Tracker + renderer geometry (pym3t: synchronize_cameras=False)
  auto tracker = std::make_shared<m3t::Tracker>("tracker", 5, 2, false);
  auto renderer_geometry =
      std::make_shared<m3t::RendererGeometry>("renderer geometry");

  // RealSense cameras. Depth camera uses the color frame as world by default,
  // which matches pym3t RealSenseDepthCamera(use_color_as_world_frame=True).
  auto color_camera =
      std::make_shared<m3t::RealSenseColorCamera>("realsense_color");
  std::shared_ptr<m3t::RealSenseDepthCamera> depth_camera;
  if (opt.use_depth) {
    depth_camera =
        std::make_shared<m3t::RealSenseDepthCamera>("realsense_depth");
  }

  auto color_viewer = std::make_shared<m3t::NormalColorViewer>(
      "color_viewer", color_camera, renderer_geometry);
  tracker->AddViewer(color_viewer);
  if (opt.use_depth && opt.use_depth_viewer) {
    auto depth_viewer = std::make_shared<m3t::NormalDepthViewer>(
        "depth_viewer_name", depth_camera, renderer_geometry);
    tracker->AddViewer(depth_viewer);
  }

  std::cout << "Loading object " << obj_path.u8string() << std::endl;
  auto body = std::make_shared<m3t::Body>(
      opt.body_name, obj_path, opt.scale_geometry,
      /*geometry_counterclockwise=*/true, /*geometry_enable_culling=*/true,
      m3t::Transform3fA::Identity());
  renderer_geometry->AddBody(body);

  auto link =
      std::make_shared<m3t::Link>(opt.body_name + "_link", body);

  // Shared renderer for modeled occlusions (region + texture). Defaults match
  // pym3t FocusedBasicDepthRenderer bindings: image_size=200, z_min=0.01, z_max=5.
  std::shared_ptr<m3t::FocusedBasicDepthRenderer> focused_color_depth_renderer;
  if (opt.model_occlusions && (opt.use_region || opt.use_texture)) {
    focused_color_depth_renderer =
        std::make_shared<m3t::FocusedBasicDepthRenderer>(
            "focused_color_depth_renderer", renderer_geometry, color_camera,
            200, 0.01f, 5.0f);
    focused_color_depth_renderer->AddReferencedBody(body);
  }

  if (opt.use_region) {
    const std::filesystem::path region_model_path =
        tmp_dir / (opt.body_name + "_region_model.bin");
    auto region_model = std::make_shared<m3t::RegionModel>(
        opt.body_name + "_region_model", body, region_model_path);
    auto region_modality = std::make_shared<m3t::RegionModality>(
        opt.body_name + "_region_modality", body, color_camera, region_model);
    if (opt.model_occlusions) {
      region_modality->ModelOcclusions(focused_color_depth_renderer);
    }
    if (opt.measure_occlusions && depth_camera) {
      region_modality->MeasureOcclusions(depth_camera);
    }
    link->AddModality(region_modality);
  }

  if (opt.use_depth) {
    const std::filesystem::path depth_model_path =
        tmp_dir / (opt.body_name + "_depth_model.bin");
    auto depth_model = std::make_shared<m3t::DepthModel>(
        opt.body_name + "_depth_model", body, depth_model_path);
    auto depth_modality = std::make_shared<m3t::DepthModality>(
        opt.body_name + "_depth_modality", body, depth_camera, depth_model);
    if (opt.model_occlusions) {
      auto focused_depth_depth_renderer =
          std::make_shared<m3t::FocusedBasicDepthRenderer>(
              "focused_depth_depth_renderer", renderer_geometry, depth_camera,
              200, 0.01f, 5.0f);
      focused_depth_depth_renderer->AddReferencedBody(body);
      depth_modality->ModelOcclusions(focused_depth_depth_renderer);
    }
    if (opt.measure_occlusions) {
      depth_modality->MeasureOcclusions();
    }
    link->AddModality(depth_modality);
  }

  if (opt.use_texture) {
    auto color_silhouette_renderer =
        std::make_shared<m3t::FocusedSilhouetteRenderer>(
            "color_silhouette_renderer", renderer_geometry, color_camera,
            m3t::IDType::BODY, 200, 0.01f, 5.0f);
    color_silhouette_renderer->AddReferencedBody(body);
    auto texture_modality = std::make_shared<m3t::TextureModality>(
        opt.body_name + "_texture_modality", body, color_camera,
        color_silhouette_renderer);
    if (opt.model_occlusions) {
      texture_modality->ModelOcclusions(focused_color_depth_renderer);
    }
    if (opt.measure_occlusions && depth_camera) {
      texture_modality->MeasureOcclusions(depth_camera);
    }
    link->AddModality(texture_modality);
  }

  auto optimizer = std::make_shared<m3t::Optimizer>(
      opt.body_name + "_optimizer", link);
  tracker->AddOptimizer(optimizer);

  if (opt.print_pose) {
    tracker->AddPublisher(
        std::make_shared<PosePrinter>("pose_printer", body));
  }

  if (!tracker->SetUp()) {
    std::cerr << "tracker SetUp failed" << std::endl;
    return -1;
  }

  const m3t::Transform3fA body2world_pose = MakeDefaultBody2WorldPose();
  auto detector = std::make_shared<m3t::StaticDetector>(
      "static_detector", optimizer, body2world_pose,
      /*reset_joint_poses=*/false);
  tracker->AddDetector(detector);

  if (!tracker->SetUp()) {
    std::cerr << "tracker SetUp failed after adding detector" << std::endl;
    return -1;
  }
  std::cout << "tracker.SetUp ok: 1" << std::endl;

  // pym3t passes {body_name}, but Tracker::ValidateNames() only keeps
  // optimizer names. Use the optimizer name so detection/tracking actually
  // start (press D / X in the viewer if you need to re-initialize).
  const std::set<std::string> names_detecting{optimizer->name()};
  const std::set<std::string> names_starting{optimizer->name()};
  if (!tracker->RunTrackerProcess(/*execute_detection=*/true,
                                  /*start_tracking=*/true, &names_detecting,
                                  &names_starting)) {
    std::cerr << "RunTrackerProcess failed" << std::endl;
    return -1;
  }
  return 0;
}
