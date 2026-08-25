#include "apps/stack_filter/box_cloud.h"
#include "apps/stack_filter/self_test.h"
#include "stack_filter/stacked_object_filter.h"

#include <nlohmann/json.hpp>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

using json = nlohmann::json;

namespace {

void PrintUsage(const char* argv0) {
	std::cerr
		<< "Usage:\n"
		<< "  " << argv0 << " --self-test\n"
		<< "  " << argv0 << " --demo [--method projection_2d|bounding_box_3d|highest_layer]\n"
		<< "                 [--out-dir DIR]\n"
		<< "  " << argv0 << " --template MODEL.pcd --poses POSES.json [--out-dir DIR]\n"
		<< "                 [--method projection_2d|bounding_box_3d|highest_layer]\n"
		<< "                 [--threshold 0.30] [--pixel-size 0.0025]\n"
		<< "                 [--height-margin 0.003]\n"
		<< "\n"
		<< "poses.json schema:\n"
		<< "  {\n"
		<< "    \"method\": \"projection_2d\",\n"
		<< "    \"overlap_ratio_threshold\": 0.3,\n"
		<< "    \"up_axis\": [0, 0, 1],\n"
		<< "    \"targets\": [\n"
		<< "      {\"id\": \"A\", \"translation\": [0, 0, 0.025],\n"
		<< "       \"quaternion_wxyz\": [1, 0, 0, 0], \"score\": 0.9},\n"
		<< "      {\"id\": \"B\", \"matrix\": [1,0,0,0.1, 0,1,0,0, 0,0,1,0.025, 0,0,0,1]}\n"
		<< "    ]\n"
		<< "  }\n";
}

Eigen::Matrix4f ParsePose(const json& node) {
	Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
	if (node.contains("matrix")) {
		const auto& m = node["matrix"];
		if (m.is_array() && m.size() == 16) {
			for (int r = 0; r < 4; ++r) {
				for (int c = 0; c < 4; ++c) {
					pose(r, c) = m.at(static_cast<std::size_t>(r * 4 + c)).get<float>();
				}
			}
			return pose;
		}
		if (m.is_array() && m.size() == 4 && m[0].is_array()) {
			for (int r = 0; r < 4; ++r) {
				for (int c = 0; c < 4; ++c) {
					pose(r, c) = m.at(r).at(c).get<float>();
				}
			}
			return pose;
		}
	}
	if (node.contains("translation") && node["translation"].is_array()
		&& node["translation"].size() >= 3) {
		pose(0, 3) = node["translation"][0].get<float>();
		pose(1, 3) = node["translation"][1].get<float>();
		pose(2, 3) = node["translation"][2].get<float>();
	}
	if (node.contains("quaternion_wxyz") && node["quaternion_wxyz"].is_array()
		&& node["quaternion_wxyz"].size() >= 4) {
		const float w = node["quaternion_wxyz"][0].get<float>();
		const float x = node["quaternion_wxyz"][1].get<float>();
		const float y = node["quaternion_wxyz"][2].get<float>();
		const float z = node["quaternion_wxyz"][3].get<float>();
		Eigen::Quaternionf q(w, x, y, z);
		pose.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
	}
	return pose;
}

json LoadPosesFile(
	const std::string& path,
	poser::StackFilterParams* params,
	std::vector<poser::RegisteredInstance>* instances
) {
	std::ifstream ifs(path);
	if (!ifs) {
		throw std::runtime_error("Failed to open poses JSON: " + path);
	}
	json root;
	ifs >> root;

	if (root.contains("method")) {
		poser::StackFilterMethod method;
		if (!poser::ParseStackFilterMethod(root["method"].get<std::string>(), &method)) {
			throw std::runtime_error("Unknown method in JSON: " + root["method"].dump());
		}
		params->method = method;
	}
	if (root.contains("overlap_ratio_threshold")) {
		params->overlap_ratio_threshold = root["overlap_ratio_threshold"].get<float>();
	}
	if (root.contains("pixel_size")) {
		params->pixel_size = root["pixel_size"].get<float>();
	}
	if (root.contains("voxel_size")) {
		params->voxel_size = root["voxel_size"].get<float>();
	}
	if (root.contains("height_margin")) {
		params->height_margin = root["height_margin"].get<float>();
	}
	if (root.contains("layer_thickness")) {
		params->layer_thickness = root["layer_thickness"].get<float>();
	}
	if (root.contains("up_axis") && root["up_axis"].is_array() && root["up_axis"].size() >= 3) {
		params->up_axis = Eigen::Vector3f(
			root["up_axis"][0].get<float>(),
			root["up_axis"][1].get<float>(),
			root["up_axis"][2].get<float>());
	}

	if (!root.contains("targets") || !root["targets"].is_array()) {
		throw std::runtime_error("poses JSON missing targets[]");
	}
	for (const auto& node : root["targets"]) {
		poser::RegisteredInstance inst;
		inst.id = node.value("id", std::string("obj_") + std::to_string(instances->size()));
		inst.score = node.value("score", 1.0f);
		inst.pose = ParsePose(node);
		instances->push_back(inst);
	}
	return root;
}

void SaveColoredPly(
	const std::string& path,
	const std::vector<poser::RegisteredInstance>& instances,
	const poser::StackFilterOutput& output
) {
	std::vector<pcl::PointXYZ> pts;
	std::vector<std::array<std::uint8_t, 3>> rgb;
	for (std::size_t i = 0; i < instances.size(); ++i) {
		if (!instances[i].model) {
			continue;
		}
		auto world = poser::StackedObjectFilter::TransformModel(
			*instances[i].model, instances[i].pose);
		const bool kept = output.instances[i].kept;
		const std::array<std::uint8_t, 3> color = kept
			? std::array<std::uint8_t, 3>{32, 180, 80}
			: std::array<std::uint8_t, 3>{210, 50, 50};
		for (const auto& p : world->points) {
			pts.push_back(p);
			rgb.push_back(color);
		}
	}

	std::ofstream ofs(path);
	ofs << "ply\nformat ascii 1.0\n";
	ofs << "element vertex " << pts.size() << "\n";
	ofs << "property float x\nproperty float y\nproperty float z\n";
	ofs << "property uchar red\nproperty uchar green\nproperty uchar blue\n";
	ofs << "end_header\n";
	ofs << std::fixed << std::setprecision(6);
	for (std::size_t i = 0; i < pts.size(); ++i) {
		ofs << pts[i].x << " " << pts[i].y << " " << pts[i].z << " "
		    << static_cast<int>(rgb[i][0]) << " "
		    << static_cast<int>(rgb[i][1]) << " "
		    << static_cast<int>(rgb[i][2]) << "\n";
	}
}

void SaveTopDownSvg(
	const std::string& path,
	const std::vector<poser::RegisteredInstance>& instances,
	const poser::StackFilterOutput& output
) {
	struct Box {
		float minx, miny, maxx, maxy;
		std::string label;
		bool kept;
	};
	std::vector<Box> boxes;
	float gminx = 1e9f, gminy = 1e9f, gmaxx = -1e9f, gmaxy = -1e9f;
	for (std::size_t i = 0; i < instances.size(); ++i) {
		if (!instances[i].model || instances[i].model->empty()) {
			continue;
		}
		auto world = poser::StackedObjectFilter::TransformModel(
			*instances[i].model, instances[i].pose);
		Eigen::Vector4f mn, mx;
		pcl::getMinMax3D(*world, mn, mx);
		Box b;
		b.minx = mn.x();
		b.miny = mn.y();
		b.maxx = mx.x();
		b.maxy = mx.y();
		b.kept = output.instances[i].kept;
		std::ostringstream label;
		label << output.instances[i].id << " "
		      << (b.kept ? "KEEP" : "DROP") << " "
		      << std::fixed << std::setprecision(2)
		      << output.instances[i].overlap_ratio;
		b.label = label.str();
		boxes.push_back(b);
		gminx = std::min(gminx, b.minx);
		gminy = std::min(gminy, b.miny);
		gmaxx = std::max(gmaxx, b.maxx);
		gmaxy = std::max(gmaxy, b.maxy);
	}
	if (boxes.empty()) {
		return;
	}

	const float pad = 0.02f;
	gminx -= pad;
	gminy -= pad;
	gmaxx += pad;
	gmaxy += pad;
	const float w = std::max(gmaxx - gminx, 1e-3f);
	const float h = std::max(gmaxy - gminy, 1e-3f);
	const int svg_w = 720;
	const int svg_h = static_cast<int>(std::round(720.0f * h / w));
	auto sx = [&](float x) { return (x - gminx) / w * svg_w; };
	auto sy = [&](float y) { return svg_h - (y - gminy) / h * svg_h; };

	std::ofstream ofs(path);
	ofs << "<svg xmlns='http://www.w3.org/2000/svg' width='" << svg_w
	    << "' height='" << svg_h << "' viewBox='0 0 " << svg_w << " " << svg_h << "'>\n";
	ofs << "<rect width='100%' height='100%' fill='#111'/>\n";
	ofs << "<text x='16' y='24' fill='#ddd' font-size='16' font-family='sans-serif'>"
	    << "Stack filter top-down (green=KEEP / red=DROP)</text>\n";
	for (const auto& b : boxes) {
		const float x = sx(b.minx);
		const float y = sy(b.maxy);
		const float bw = sx(b.maxx) - sx(b.minx);
		const float bh = sy(b.miny) - sy(b.maxy);
		const char* fill = b.kept ? "#20b450" : "#d23232";
		ofs << "<rect x='" << x << "' y='" << y << "' width='" << bw
		    << "' height='" << bh << "' fill='" << fill
		    << "' fill-opacity='0.35' stroke='" << fill << "' stroke-width='2'/>\n";
		ofs << "<text x='" << (x + 6) << "' y='" << (y + 18)
		    << "' fill='#fff' font-size='13' font-family='sans-serif'>"
		    << b.label << "</text>\n";
	}
	ofs << "</svg>\n";
}

json ResultToJson(
	const poser::StackFilterParams& params,
	const poser::StackFilterOutput& output
) {
	json root;
	root["method"] = poser::StackedObjectFilter::MethodName(params.method);
	root["overlap_ratio_threshold"] = params.overlap_ratio_threshold;
	root["kept_count"] = output.kept_indices.size();
	root["total_count"] = output.instances.size();
	root["kept_indices"] = output.kept_indices;
	json arr = json::array();
	for (const auto& inst : output.instances) {
		json item;
		item["index"] = inst.index;
		item["id"] = inst.id;
		item["score"] = inst.score;
		item["overlap_ratio"] = inst.overlap_ratio;
		item["mean_height"] = inst.mean_height;
		item["occupied_cells"] = inst.occupied_cells;
		item["pressed_cells"] = inst.pressed_cells;
		item["kept"] = inst.kept;
		arr.push_back(item);
	}
	root["instances"] = arr;
	return root;
}

void PrintTable(const poser::StackFilterOutput& output) {
	std::cout << std::left << std::setw(6) << "idx"
	          << std::setw(14) << "id"
	          << std::setw(12) << "height"
	          << std::setw(10) << "overlap"
	          << std::setw(10) << "cells"
	          << "flag\n";
	for (const auto& inst : output.instances) {
		std::cout << std::left << std::setw(6) << inst.index
		          << std::setw(14) << inst.id
		          << std::setw(12) << std::fixed << std::setprecision(4) << inst.mean_height
		          << std::setw(10) << std::setprecision(3) << inst.overlap_ratio
		          << inst.pressed_cells << "/" << inst.occupied_cells << "\t"
		          << (inst.kept ? "KEEP" : "DROP") << "\n";
	}
	std::cout << "kept (high to low):";
	for (int idx : output.kept_indices) {
		std::cout << " " << output.instances[idx].id;
	}
	std::cout << "\n";
}

std::vector<poser::RegisteredInstance> BuildDemoScene(
	pcl::PointCloud<pcl::PointXYZ>::Ptr* model_out
) {
	const float sx = 0.10f;
	const float sy = 0.10f;
	const float sz = 0.05f;
	auto model = poser::demo::MakeBoxSurfaceCloud(sx, sy, sz, 0.004f);
	if (model_out) {
		*model_out = model;
	}

	// Two piles plus one free object:
	//   A (bottom) pressed by C (top)
	//   B (bottom) partly pressed by D (offset top)
	//   E free on the table — kept even though globally lower than C/D
	std::vector<std::pair<std::string, Eigen::Matrix4f>> poses = {
		{"A", poser::demo::MakePose(0.00f, 0.00f, 0.5f * sz)},
		{"B", poser::demo::MakePose(0.16f, 0.00f, 0.5f * sz)},
		{"C", poser::demo::MakePose(0.00f, 0.00f, 1.5f * sz)},
		{"D", poser::demo::MakePose(0.20f, 0.02f, 1.5f * sz)},
		{"E", poser::demo::MakePose(0.40f, 0.00f, 0.5f * sz)},
	};
	std::vector<poser::RegisteredInstance> instances;
	for (const auto& item : poses) {
		poser::RegisteredInstance inst;
		inst.id = item.first;
		inst.model = model;
		inst.pose = item.second;
		instances.push_back(inst);
	}
	return instances;
}

int WriteOutputs(
	const std::string& out_dir,
	const std::vector<poser::RegisteredInstance>& instances,
	const poser::StackFilterParams& params,
	const poser::StackFilterOutput& output
) {
	const std::string ply = out_dir + "/stack_filter_result.ply";
	const std::string svg = out_dir + "/stack_filter_topdown.svg";
	const std::string js = out_dir + "/stack_filter_result.json";
	SaveColoredPly(ply, instances, output);
	SaveTopDownSvg(svg, instances, output);
	std::ofstream ofs(js);
	ofs << std::setw(2) << ResultToJson(params, output) << "\n";
	std::cout << "Wrote " << ply << "\nWrote " << svg << "\nWrote " << js << "\n";
	return 0;
}

}  // namespace

int main(int argc, char** argv) {
	std::string template_path;
	std::string poses_path;
	std::string out_dir = ".";
	bool self_test = false;
	bool demo = false;
	poser::StackFilterParams params;
	bool method_set = false;
	bool threshold_set = false;

	for (int i = 1; i < argc; ++i) {
		const std::string arg = argv[i];
		auto need = [&](const char* name) -> std::string {
			if (i + 1 >= argc) {
				throw std::runtime_error(std::string("Missing value for ") + name);
			}
			return argv[++i];
		};
		if (arg == "--self-test") {
			self_test = true;
		} else if (arg == "--demo") {
			demo = true;
		} else if (arg == "--template") {
			template_path = need("--template");
		} else if (arg == "--poses") {
			poses_path = need("--poses");
		} else if (arg == "--out-dir") {
			out_dir = need("--out-dir");
		} else if (arg == "--method") {
			if (!poser::ParseStackFilterMethod(need("--method"), &params.method)) {
				std::cerr << "Unknown method\n";
				return 2;
			}
			method_set = true;
		} else if (arg == "--threshold") {
			params.overlap_ratio_threshold = std::stof(need("--threshold"));
			threshold_set = true;
		} else if (arg == "--pixel-size") {
			params.pixel_size = std::stof(need("--pixel-size"));
		} else if (arg == "--height-margin") {
			params.height_margin = std::stof(need("--height-margin"));
		} else if (arg == "--help" || arg == "-h") {
			PrintUsage(argv[0]);
			return 0;
		} else {
			std::cerr << "Unknown argument: " << arg << "\n";
			PrintUsage(argv[0]);
			return 2;
		}
	}

	if (self_test) {
		return poser::RunStackFilterSelfTest();
	}

	try {
		std::vector<poser::RegisteredInstance> instances;
		pcl::PointCloud<pcl::PointXYZ>::Ptr model;

		if (demo) {
			instances = BuildDemoScene(&model);
			params.pixel_size = (params.pixel_size > 0.0f) ? params.pixel_size : 0.005f;
		} else {
			if (template_path.empty() || poses_path.empty()) {
				PrintUsage(argv[0]);
				return 2;
			}
			model.reset(new pcl::PointCloud<pcl::PointXYZ>());
			if (pcl::io::loadPCDFile(template_path, *model) < 0 || model->empty()) {
				std::cerr << "Failed to load template PCD: " << template_path << "\n";
				return 1;
			}
			poser::StackFilterParams json_params = params;
			LoadPosesFile(poses_path, &json_params, &instances);
			if (!method_set) {
				params.method = json_params.method;
			}
			if (!threshold_set) {
				params.overlap_ratio_threshold = json_params.overlap_ratio_threshold;
			}
			if (params.pixel_size <= 0.0f) {
				params.pixel_size = json_params.pixel_size;
			}
			if (params.height_margin == 0.003f) {
				params.height_margin = json_params.height_margin;
			}
			params.voxel_size = json_params.voxel_size;
			params.layer_thickness = json_params.layer_thickness;
			params.up_axis = json_params.up_axis;
			for (auto& inst : instances) {
				inst.model = model;
			}
		}

		poser::StackedObjectFilter filter(params);
		const auto output = filter.Filter(instances);
		std::cout << "method=" << poser::StackedObjectFilter::MethodName(params.method)
		          << " threshold=" << params.overlap_ratio_threshold
		          << " kept=" << output.kept_indices.size()
		          << "/" << output.instances.size() << "\n";
		PrintTable(output);
		return WriteOutputs(out_dir, instances, params, output);
	} catch (const std::exception& ex) {
		std::cerr << "Error: " << ex.what() << "\n";
		return 1;
	}
}
