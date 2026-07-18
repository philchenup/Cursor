#include "visualizer/debug_visualizer.h"

#include <glog/logging.h>

#include <fstream>

namespace poser {
namespace {

void WritePLY(const TensorView<float4>& cloud, const std::string& path) {
	std::ofstream ofs(path);
	LOG_ASSERT(ofs.is_open()) << "Failed to write PLY: " << path;
	ofs << "ply\nformat ascii 1.0\n";
	ofs << "element vertex " << cloud.Size() << "\n";
	ofs << "property float x\nproperty float y\nproperty float z\n";
	ofs << "end_header\n";
	for (auto i = 0u; i < cloud.Size(); ++i) {
		const auto& p = cloud[i];
		ofs << p.x << " " << p.y << " " << p.z << "\n";
	}
	LOG(INFO) << "Saved point cloud (" << cloud.Size() << " pts) to " << path;
}

}  // namespace

void DebugVisualizer::SaveCloudPairPLY(
	const TensorView<float4>& cloud_1,
	const TensorView<float4>& cloud_2,
	const std::string& path_1,
	const std::string& path_2
) {
	WritePLY(cloud_1, path_1);
	WritePLY(cloud_2, path_2);
}

void DebugVisualizer::DrawMatchedCloudPair(
	const TensorView<float4>& cloud_1,
	const TensorView<float4>& cloud_2
) {
	// Keep the original call site API; export PLY instead of PCL visualizer.
	SaveCloudPairPLY(cloud_1, cloud_2, "matched_live.ply", "matched_observation.ply");
}

}  // namespace poser
