#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <vector>

namespace poser {

// Millimetres. Viewpoint is on the -Z side, looking toward +Z.
// Smaller Z is closer. An object is dropped only if another, closer
// object covers enough of its XY footprint (overlap > threshold).
// Isolated objects are kept even when they sit farther along Z.
// BoundingBox3D fills a PCA-aligned cuboid, not the world AABB.

enum class StackFilterMethod {
	Projection2D = 0,
	BoundingBox3D = 1,
};

struct StackFilterResult {
	std::vector<int> kept;             // not stacked; closer first
	std::vector<float> overlap_ratio;  // stacked XY/volume fraction in [0, 1]
};

StackFilterResult FilterStacked(
	const std::vector<pcl::PointCloud<pcl::PointXYZ>>& clouds,
	StackFilterMethod method = StackFilterMethod::Projection2D,
	float overlap_threshold = 0.1f);

}  // namespace poser
