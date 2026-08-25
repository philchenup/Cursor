#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <vector>

namespace poser {

// Input clouds are already in the camera/world frame, millimetres.
// Camera +Z points into the scene (down): smaller Z is closer / on top.
// overlap_ratio = XY area covered by a closer cloud / this cloud's XY area.

struct StackFilterResult {
	std::vector<int> kept;              // uncovered top-layer indices, near-camera first
	std::vector<float> overlap_ratio;   // one value per input cloud in [0, 1]
};

StackFilterResult FilterStacked(
	const std::vector<pcl::PointCloud<pcl::PointXYZ>>& clouds,
	float overlap_threshold = 0.1f);

}  // namespace poser
