#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <vector>

namespace poser {

// Millimetres. Viewpoint is on the -Z side, looking toward +Z.
// Depth along the view is Z: smaller Z is closer / top layer.
// Keep only top-layer objects that are not stacked.
// BoundingBox3D fills a PCA-aligned cuboid (not the world AABB) so a tilted
// part cannot steal the whole top layer from one close corner.

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
