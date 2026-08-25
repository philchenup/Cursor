#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <vector>

namespace poser {

// Viewpoint is the origin, looking along +Z (millimetres).
// Smaller Z is closer to the camera and counts as the top layer.
// An object is kept only if it is on that top layer and not stacked.

enum class StackFilterMethod {
	Projection2D = 0,
	BoundingBox3D = 1,
};

struct StackFilterResult {
	std::vector<int> kept;             // top-layer, not stacked; closer first
	std::vector<float> overlap_ratio;  // stacked XY/volume fraction in [0, 1]
};

StackFilterResult FilterStacked(
	const std::vector<pcl::PointCloud<pcl::PointXYZ>>& clouds,
	StackFilterMethod method = StackFilterMethod::Projection2D,
	float overlap_threshold = 0.1f);

}  // namespace poser
