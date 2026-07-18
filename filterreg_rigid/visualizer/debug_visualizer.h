#pragma once

#include "common/common_type.h"
#include "common/tensor_access.h"

#include <string>

namespace poser {

// PCL-free debug helper: export matched clouds to PLY instead of opening a GUI.
class DebugVisualizer {
public:
	static void DrawMatchedCloudPair(
		const TensorView<float4>& cloud_1,
		const TensorView<float4>& cloud_2);

	static void SaveCloudPairPLY(
		const TensorView<float4>& cloud_1,
		const TensorView<float4>& cloud_2,
		const std::string& path_1,
		const std::string& path_2);
};

}  // namespace poser
