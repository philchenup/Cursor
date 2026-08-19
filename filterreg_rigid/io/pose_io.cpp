#include "io/pose_io.h"

#include <glog/logging.h>

#include <fstream>
#include <vector>

namespace poser {
namespace {

bool ParseXyzwQuaternion(const json& q_json, Eigen::Quaterniond& q) {
	if (!q_json.is_array() || q_json.size() != 4) {
		LOG(ERROR) << "Quaternion must be an array of 4 numbers [x, y, z, w]";
		return false;
	}
	const double x = q_json.at(0).get<double>();
	const double y = q_json.at(1).get<double>();
	const double z = q_json.at(2).get<double>();
	const double w = q_json.at(3).get<double>();
	q = Eigen::Quaterniond(w, x, y, z);
	if (q.norm() < 1e-12) {
		LOG(ERROR) << "Quaternion has near-zero norm";
		return false;
	}
	q.normalize();
	return true;
}

bool ParseTranslation(const json& t_json, Eigen::Vector3d& t) {
	if (!t_json.is_array() || t_json.size() != 3) {
		LOG(ERROR) << "Translation must be an array of 3 numbers [tx, ty, tz]";
		return false;
	}
	t.x() = t_json.at(0).get<double>();
	t.y() = t_json.at(1).get<double>();
	t.z() = t_json.at(2).get<double>();
	return true;
}

}  // namespace

bool Affine3dFromJson(const json& j, Eigen::Affine3d& pose, int* eih) {
	if (!j.is_object()) {
		LOG(ERROR) << "Hand-eye JSON must be an object";
		return false;
	}
	if (!j.contains("Quaternion") || !j.contains("Translation")) {
		LOG(ERROR) << "Hand-eye JSON requires Quaternion and Translation fields";
		return false;
	}

	Eigen::Quaterniond q;
	Eigen::Vector3d t;
	if (!ParseXyzwQuaternion(j.at("Quaternion"), q)) return false;
	if (!ParseTranslation(j.at("Translation"), t)) return false;

	pose = Eigen::Affine3d::Identity();
	pose.translate(t);
	pose.rotate(q);

	if (eih != nullptr) {
		*eih = j.value("EIH", 0);
	}
	return true;
}

bool HandEyeCalibFromJson(const json& j, HandEyeCalib& calib) {
	return Affine3dFromJson(j, calib.pose, &calib.eih);
}

bool LoadHandEyeCalibFromJsonFile(const std::string& path, HandEyeCalib& calib) {
	std::ifstream ifs(path);
	if (!ifs) {
		LOG(ERROR) << "Failed to open JSON file: " << path;
		return false;
	}

	json j;
	try {
		ifs >> j;
	} catch (const json::exception& e) {
		LOG(ERROR) << "Failed to parse JSON file " << path << ": " << e.what();
		return false;
	}

	if (!HandEyeCalibFromJson(j, calib)) {
		LOG(ERROR) << "Failed to convert JSON pose: " << path;
		return false;
	}
	return true;
}

bool LoadAffine3dFromJsonFile(const std::string& path, Eigen::Affine3d& pose, int* eih) {
	HandEyeCalib calib;
	if (!LoadHandEyeCalibFromJsonFile(path, calib)) return false;
	pose = calib.pose;
	if (eih != nullptr) *eih = calib.eih;
	return true;
}

}  // namespace poser
