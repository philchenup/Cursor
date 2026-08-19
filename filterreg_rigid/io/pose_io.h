#pragma once

#include "common/common_type.h"

#include <string>

namespace poser {

// Hand-eye calibration JSON (nlohmann), matching files such as:
//   HandOnEyeCalib-YYYY-MM-DD-HH-MM-SS_xxxx.json
//
// {
//   "EIH": 2,
//   "Quaternion": [x, y, z, w],   // Eigen coeffs() / ROS xyzw
//   "Translation": [tx, ty, tz]   // same unit as the calibration dump (often mm)
// }
struct HandEyeCalib {
	int eih = 0;
	Eigen::Affine3d pose = Eigen::Affine3d::Identity();
};

// Convert an already-parsed nlohmann JSON object to Eigen::Affine3d.
// Quaternion is interpreted as [x, y, z, w] and normalized.
bool Affine3dFromJson(const json& j, Eigen::Affine3d& pose, int* eih = nullptr);

// Same conversion, also returning the optional EIH flag.
bool HandEyeCalibFromJson(const json& j, HandEyeCalib& calib);

// Read a HandOnEyeCalib JSON file with nlohmann::json and convert to Affine3d.
bool LoadAffine3dFromJsonFile(const std::string& path, Eigen::Affine3d& pose, int* eih = nullptr);
bool LoadHandEyeCalibFromJsonFile(const std::string& path, HandEyeCalib& calib);

}  // namespace poser
