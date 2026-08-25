#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <vector>

namespace poser {

// Camera-frame clouds in millimetres, camera +Z down (smaller Z = closer).
// Keep only the global top layer, and only those not pressed from above.
// A lower-layer object is never graspable, even if nothing sits on it.

struct StackFilterResult {
	std::vector<int> kept;              // top-layer, uncovered; near-camera first
	std::vector<float> overlap_ratio;   // XY fraction covered by a closer cloud
};

StackFilterResult FilterStacked(
	const std::vector<pcl::PointCloud<pcl::PointXYZ>>& clouds,
	float overlap_threshold = 0.1f);

}  // namespace poser
