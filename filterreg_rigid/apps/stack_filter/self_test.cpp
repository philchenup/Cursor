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
		          << " ratio=" << inst.overlap_ratio << "\n";
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
		          << " ratio=" << inst.overlap_ratio << "\n";
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

}  // namespace

int RunStackFilterSelfTest() {
	const float sx = 0.10f;
	const float sy = 0.10f;
	const float sz = 0.05f;
	auto model = demo::MakeBoxSurfaceCloud(sx, sy, sz, 0.004f);

	StackFilterParams params;
	params.method = StackFilterMethod::Projection2D;
	params.overlap_ratio_threshold = 0.30f;
	params.pixel_size = 0.005f;
	params.height_margin = 0.003f;
	params.downsample = true;
	StackedObjectFilter filter(params);

	int failures = 0;

	{
		auto instances = MakeInstances(model, {
			{"A", demo::MakePose(0.00f, 0.00f, 0.5f * sz)},
			{"B", demo::MakePose(0.16f, 0.00f, 0.5f * sz)},
		});
		const auto out = filter.Filter(instances);
		if (!ExpectKeep("side_by_side", out, 0, true, 0.0f, 0.05f)) ++failures;
		if (!ExpectKeep("side_by_side", out, 1, true, 0.0f, 0.05f)) ++failures;
	}

	{
		auto instances = MakeInstances(model, {
			{"bottom", demo::MakePose(0.00f, 0.00f, 0.5f * sz)},
			{"top", demo::MakePose(0.00f, 0.00f, 1.5f * sz)},
		});
		const auto out = filter.Filter(instances);
		if (!ExpectKeep("full_stack", out, 0, false, 0.70f, 1.01f)) ++failures;
		if (!ExpectKeep("full_stack", out, 1, true, 0.0f, 0.05f)) ++failures;
	}

	{
		auto instances = MakeInstances(model, {
			{"base", demo::MakePose(0.00f, 0.00f, 0.5f * sz)},
			{"offset_top", demo::MakePose(0.04f, 0.00f, 1.5f * sz)},
		});
		const auto out = filter.Filter(instances);
		if (!ExpectKeep("partial_stack_base", out, 0, false, 0.35f, 0.85f)) ++failures;
		if (!ExpectKeep("partial_stack_top", out, 1, true, 0.0f, 0.08f)) ++failures;
	}

	{
		auto instances = MakeInstances(model, {
			{"low_top", demo::MakePose(0.00f, 0.00f, 0.5f * sz)},
			{"high_bottom", demo::MakePose(0.20f, 0.00f, 0.5f * sz)},
			{"high_top", demo::MakePose(0.20f, 0.00f, 1.5f * sz)},
		});
		const auto out = filter.Filter(instances);
		if (!ExpectKeep("two_piles_low", out, 0, true, 0.0f, 0.05f)) ++failures;
		if (!ExpectKeep("two_piles_high_bottom", out, 1, false, 0.70f, 1.01f)) ++failures;
		if (!ExpectKeep("two_piles_high_top", out, 2, true, 0.0f, 0.05f)) ++failures;
	}

	{
		StackedObjectFilter bbox(params);
		StackFilterParams p = params;
		p.method = StackFilterMethod::BoundingBox3D;
		p.voxel_size = 0.008f;
		p.expand_z = 1.0f;
		bbox.set_params(p);
		auto instances = MakeInstances(model, {
			{"bottom", demo::MakePose(0.00f, 0.00f, 0.5f * sz)},
			{"top", demo::MakePose(0.00f, 0.00f, 1.5f * sz)},
		});
		const auto out = bbox.Filter(instances);
		if (!ExpectKeep("bbox3d_stack", out, 0, false, 0.50f, 1.01f)) ++failures;
		if (!ExpectKeep("bbox3d_stack", out, 1, true, 0.0f, 0.15f)) ++failures;
	}

	{
		StackFilterParams p = params;
		p.method = StackFilterMethod::HighestLayer;
		p.layer_thickness = sz;
		StackedObjectFilter height_filter(p);
		auto instances = MakeInstances(model, {
			{"low", demo::MakePose(0.00f, 0.00f, 0.5f * sz)},
			{"high", demo::MakePose(0.20f, 0.00f, 1.5f * sz)},
		});
		const auto out = height_filter.Filter(instances);
		if (!ExpectKeep("highest_layer_low", out, 0, false, 0.0f, 0.05f)) ++failures;
		if (!ExpectKeep("highest_layer_high", out, 1, true, 0.0f, 0.05f)) ++failures;
	}

	{
		auto instances = MakeInstances(model, {
			{"only", demo::MakePose(0.0f, 0.0f, 0.5f * sz)},
		});
		const auto out = filter.Filter(instances);
		if (!ExpectKeep("single", out, 0, true, 0.0f, 0.01f)) ++failures;
		if (out.kept_indices.size() != 1) {
			std::cerr << "[FAIL] single: expected 1 kept index\n";
			++failures;
		}
	}

	if (failures == 0) {
		std::cout << "All stack_filter self-tests passed.\n";
		return 0;
	}
	std::cerr << failures << " stack_filter self-test(s) failed.\n";
	return 1;
}

}  // namespace poser
