#include "apps/stack_filter/self_test.h"

#include "apps/stack_filter/box_cloud.h"

#include <cmath>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

namespace poser {
namespace {

bool ExpectKeep(
	const char* name,
	const StackFilterOutput& output,
	int index,
	bool should_keep,
	float min_ratio,
	float max_ratio
) {
	if (index < 0 || index >= static_cast<int>(output.instances.size())) {
		std::cerr << "[FAIL] " << name << ": index " << index << " out of range\n";
		return false;
	}
	const auto& inst = output.instances[index];
	bool ok = true;
	if (inst.kept != should_keep) {
		std::cerr << "[FAIL] " << name << " id=" << inst.id
		          << " expected " << (should_keep ? "KEEP" : "DROP")
		          << " got " << (inst.kept ? "KEEP" : "DROP")
		          << " ratio=" << inst.overlap_ratio
		          << " z=" << inst.mean_z << "\n";
		ok = false;
	}
	if (inst.overlap_ratio < min_ratio || inst.overlap_ratio > max_ratio) {
		std::cerr << "[FAIL] " << name << " id=" << inst.id
		          << " ratio " << inst.overlap_ratio
		          << " not in [" << min_ratio << ", " << max_ratio << "]\n";
		ok = false;
	}
	if (ok) {
		std::cout << "[PASS] " << name << " id=" << inst.id
		          << " " << (inst.kept ? "KEEP" : "DROP")
		          << " ratio=" << inst.overlap_ratio
		          << " z=" << inst.mean_z << "\n";
	}
	return ok;
}

std::vector<RegisteredInstance> MakeInstances(
	const pcl::PointCloud<pcl::PointXYZ>::Ptr& model,
	const std::vector<std::pair<std::string, Eigen::Matrix4f>>& poses
) {
	std::vector<RegisteredInstance> instances;
	instances.reserve(poses.size());
	for (const auto& item : poses) {
		RegisteredInstance inst;
		inst.id = item.first;
		inst.model = model;
		inst.pose = item.second;
		instances.push_back(inst);
	}
	return instances;
}

StackFilterParams CameraMmParams() {
	StackFilterParams params;
	params.unit = LengthUnit::Millimeters;
	params.camera_z_down = true;
	params.method = StackFilterMethod::Projection2D;
	params.overlap_ratio_threshold = 0.30f;
	params.pixel_size = 2.5f;
	params.height_margin = 3.0f;
	params.dilation_radius = 1;
	params.downsample = true;
	return params;
}

}  // namespace

int RunStackFilterSelfTest() {
	const float sx = 100.0f;
	const float sy = 100.0f;
	const float sz = 20.0f;
	auto box = demo::MakeBoxSurfaceCloud(sx, sy, sz, 4.0f);
	StackedObjectFilter filter(CameraMmParams());

	int failures = 0;

	// Camera +Z down: smaller Z is closer to the camera / on top.
	const float z_top = 380.0f;
	const float z_bottom = 400.0f;

	{
		auto instances = MakeInstances(box, {
			{"A", demo::MakePose(0.0f, 0.0f, z_bottom)},
			{"B", demo::MakePose(160.0f, 0.0f, z_bottom)},
		});
		const auto out = filter.Filter(instances);
		if (!ExpectKeep("side_by_side", out, 0, true, 0.0f, 0.08f)) ++failures;
		if (!ExpectKeep("side_by_side", out, 1, true, 0.0f, 0.08f)) ++failures;
	}

	{
		auto instances = MakeInstances(box, {
			{"bottom", demo::MakePose(0.0f, 0.0f, z_bottom)},
			{"top", demo::MakePose(0.0f, 0.0f, z_top)},
		});
		const auto out = filter.Filter(instances);
		if (!ExpectKeep("full_stack", out, 0, false, 0.70f, 1.01f)) ++failures;
		if (!ExpectKeep("full_stack", out, 1, true, 0.0f, 0.08f)) ++failures;
		if (out.instances[1].mean_z >= out.instances[0].mean_z) {
			std::cerr << "[FAIL] full_stack: top should have smaller camera Z\n";
			++failures;
		}
	}

	{
		auto instances = MakeInstances(box, {
			{"base", demo::MakePose(0.0f, 0.0f, z_bottom)},
			{"offset_top", demo::MakePose(40.0f, 0.0f, z_top)},
		});
		const auto out = filter.Filter(instances);
		if (!ExpectKeep("partial_stack_base", out, 0, false, 0.30f, 0.90f)) ++failures;
		if (!ExpectKeep("partial_stack_top", out, 1, true, 0.0f, 0.10f)) ++failures;
	}

	{
		auto instances = MakeInstances(box, {
			{"low_top", demo::MakePose(0.0f, 0.0f, z_bottom)},
			{"high_bottom", demo::MakePose(200.0f, 0.0f, z_bottom)},
			{"high_top", demo::MakePose(200.0f, 0.0f, z_top)},
		});
		const auto out = filter.Filter(instances);
		if (!ExpectKeep("two_piles_low", out, 0, true, 0.0f, 0.08f)) ++failures;
		if (!ExpectKeep("two_piles_high_bottom", out, 1, false, 0.70f, 1.01f)) ++failures;
		if (!ExpectKeep("two_piles_high_top", out, 2, true, 0.0f, 0.08f)) ++failures;
	}

	{
		StackFilterParams p = CameraMmParams();
		p.method = StackFilterMethod::BoundingBox3D;
		p.voxel_size = 8.0f;
		p.expand_z = 1.0f;
		StackedObjectFilter bbox(p);
		auto instances = MakeInstances(box, {
			{"bottom", demo::MakePose(0.0f, 0.0f, z_bottom)},
			{"top", demo::MakePose(0.0f, 0.0f, z_top)},
		});
		const auto out = bbox.Filter(instances);
		if (!ExpectKeep("bbox3d_stack", out, 0, false, 0.50f, 1.01f)) ++failures;
		if (!ExpectKeep("bbox3d_stack", out, 1, true, 0.0f, 0.20f)) ++failures;
	}

	{
		StackFilterParams p = CameraMmParams();
		p.method = StackFilterMethod::HighestLayer;
		p.layer_thickness = sz;
		StackedObjectFilter height_filter(p);
		auto instances = MakeInstances(box, {
			{"far", demo::MakePose(0.0f, 0.0f, z_bottom)},
			{"near", demo::MakePose(200.0f, 0.0f, z_top)},
		});
		const auto out = height_filter.Filter(instances);
		if (!ExpectKeep("highest_layer_far", out, 0, false, 0.0f, 0.08f)) ++failures;
		if (!ExpectKeep("highest_layer_near", out, 1, true, 0.0f, 0.08f)) ++failures;
	}

	{
		auto instances = MakeInstances(box, {
			{"only", demo::MakePose(0.0f, 0.0f, z_bottom)},
		});
		const auto out = filter.Filter(instances);
		if (!ExpectKeep("single", out, 0, true, 0.0f, 0.01f)) ++failures;
	}

	{
		auto flange = demo::MakeFlangeCloud();
		auto instances = MakeInstances(flange, {
			{"center", demo::MakePose(5.0f, 5.0f, 345.0f, 0.2f)},
			{"top_left", demo::MakePose(-65.0f, 55.0f, 322.0f, -0.3f)},
			{"bot_left", demo::MakePose(-80.0f, -70.0f, 328.0f, 0.5f)},
			{"top_right", demo::MakePose(70.0f, 45.0f, 318.0f, 0.4f)},
			{"bot_right", demo::MakePose(72.0f, -5.0f, 338.0f, -0.15f)},
		});
		const auto out = filter.Filter(instances);
		if (!ExpectKeep("flange_center", out, 0, false, 0.30f, 1.01f)) ++failures;
		if (!ExpectKeep("flange_top_left", out, 1, true, 0.0f, 0.20f)) ++failures;
		if (!ExpectKeep("flange_bot_left", out, 2, true, 0.0f, 0.20f)) ++failures;
		if (!ExpectKeep("flange_top_right", out, 3, true, 0.0f, 0.20f)) ++failures;
		if (!ExpectKeep("flange_bot_right", out, 4, false, 0.25f, 1.01f)) ++failures;
	}

	if (failures == 0) {
		std::cout << "All stack_filter self-tests passed (mm, camera Z down).\n";
		return 0;
	}
	std::cerr << failures << " stack_filter self-test(s) failed.\n";
	return 1;
}

}  // namespace poser
