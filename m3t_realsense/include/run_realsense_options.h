#ifndef RUN_REALSENSE_OPTIONS_H_
#define RUN_REALSENSE_OPTIONS_H_

#include <iostream>
#include <string>
#include <vector>

struct RealSenseExampleOptions {
  std::string body_name;
  std::string models_dir;
  float scale_geometry = 0.001f;
  std::string tmp_dir = "tmp";
  bool use_region = false;
  bool use_depth = false;
  bool use_texture = false;
  bool use_depth_viewer = false;
  bool model_occlusions = false;
  bool measure_occlusions = false;
  bool print_pose = false;
};

inline void PrintRealSenseExampleUsage(const char *argv0) {
  std::cerr
      << "Usage: " << argv0 << " -b <body_name> -m <models_dir> [options]\n"
      << "\nRequired:\n"
      << "  -b, --body_name NAME     Object name; loads {models_dir}/{NAME}.obj\n"
      << "  -m, --models_dir DIR     Directory that contains {body_name}.obj\n"
      << "\nOptions (same as pym3t run_realsense_example.py):\n"
      << "  --scale_geometry F       Mesh unit to meters (default 0.001 = mm)\n"
      << "  --tmp_dir DIR            Cache for *_region_model.bin / *_depth_model.bin\n"
      << "  --use_region             Enable region (contour) modality\n"
      << "  --use_depth              Enable RealSense depth + depth modality\n"
      << "  --use_texture            Enable texture (ORB) modality\n"
      << "  --use_depth_viewer       Overlay the depth stream as well\n"
      << "  --model_occlusions       Rendered-depth occlusion handling\n"
      << "  --measure_occlusions     Measure occlusions from the depth camera\n"
      << "  --print_pose             Print body2world 4x4 each frame\n";
}

inline bool ConsumeFlag(const std::string &arg, const std::string &name,
                        bool *dst) {
  if (arg == name) {
    *dst = true;
    return true;
  }
  return false;
}

inline bool ParseRealSenseExampleArgs(const std::vector<std::string> &args,
                                      RealSenseExampleOptions *opt,
                                      std::string *error) {
  auto fail = [&](const std::string &msg) {
    if (error) *error = msg;
    return false;
  };

  for (size_t i = 1; i < args.size(); ++i) {
    const std::string &arg = args[i];
    auto need_value = [&](const char *name) -> const std::string * {
      if (i + 1 >= args.size()) {
        if (error) {
          *error = std::string("Missing value for ") + name;
        }
        return nullptr;
      }
      return &args[++i];
    };

    if (arg == "-h" || arg == "--help") {
      return fail("help");
    }
    if (arg == "-b" || arg == "--body_name") {
      const std::string *v = need_value("--body_name");
      if (!v) return false;
      opt->body_name = *v;
    } else if (arg == "-m" || arg == "--models_dir") {
      const std::string *v = need_value("--models_dir");
      if (!v) return false;
      opt->models_dir = *v;
    } else if (arg == "--scale_geometry") {
      const std::string *v = need_value("--scale_geometry");
      if (!v) return false;
      opt->scale_geometry = std::stof(*v);
    } else if (arg == "--tmp_dir") {
      const std::string *v = need_value("--tmp_dir");
      if (!v) return false;
      opt->tmp_dir = *v;
    } else if (ConsumeFlag(arg, "--use_region", &opt->use_region) ||
               ConsumeFlag(arg, "--use_depth", &opt->use_depth) ||
               ConsumeFlag(arg, "--use_texture", &opt->use_texture) ||
               ConsumeFlag(arg, "--use_depth_viewer",
                           &opt->use_depth_viewer) ||
               ConsumeFlag(arg, "--model_occlusions",
                           &opt->model_occlusions) ||
               ConsumeFlag(arg, "--measure_occlusions",
                           &opt->measure_occlusions) ||
               ConsumeFlag(arg, "--print_pose", &opt->print_pose)) {
    } else {
      return fail("Unknown argument: " + arg);
    }
  }

  if (opt->body_name.empty() || opt->models_dir.empty()) {
    return fail("Missing required -b/--body_name and -m/--models_dir");
  }
  if (!opt->use_region && !opt->use_depth && !opt->use_texture) {
    opt->use_region = true;
  }
  if (opt->measure_occlusions && !opt->use_depth) {
    return fail("--measure_occlusions requires --use_depth");
  }
  if (opt->use_depth_viewer && !opt->use_depth) {
    return fail("--use_depth_viewer requires --use_depth");
  }
  return true;
}

#endif  // RUN_REALSENSE_OPTIONS_H_
