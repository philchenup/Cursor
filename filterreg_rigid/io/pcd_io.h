#pragma once

#include "common/feature_map.h"

#include <string>
#include <vector>

namespace poser {

struct PcdPointXYZ {
	float x = 0.f, y = 0.f, z = 0.f;
};

struct PcdPointXYZNormal {
	float x = 0.f, y = 0.f, z = 0.f;
	float normal_x = 0.f, normal_y = 0.f, normal_z = 0.f;
};

// Lightweight PCD reader (ASCII / binary / binary_compressed). No PCL.
bool LoadPcdXYZ(const std::string& path, std::vector<PcdPointXYZ>& points);
bool LoadPcdXYZNormal(const std::string& path, std::vector<PcdPointXYZNormal>& points);

// Helpers used by the rigid apps.
void LoadVerticesToFeatureMap(
	FeatureMap& feature_map,
	const FeatureChannelType& channel,
	const std::vector<PcdPointXYZ>& points);

void LoadVerticesAndNormalsToFeatureMap(
	FeatureMap& feature_map,
	const FeatureChannelType& vertex_channel,
	const FeatureChannelType& normal_channel,
	const std::vector<PcdPointXYZNormal>& points);

}  // namespace poser
