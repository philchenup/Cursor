#include "io/pose_io.h"

#include <glog/logging.h>

#include <cmath>
#include <iostream>
#include <string>

namespace {

bool AlmostEqual(double a, double b, double tol = 1e-9) {
	return std::fabs(a - b) <= tol;
}

bool TestRoundTrip() {
	using namespace poser;
	json j;
	j["EIH"] = 1;
	j["Quaternion"] = json::array({0.0, 0.0, 0.0, 1.0});
	j["Translation"] = json::array({1.5, -2.0, 3.25});

	Eigen::Affine3d pose = Eigen::Affine3d::Identity();
	int eih = -1;
	if (!Affine3dFromJson(j, pose, &eih)) return false;
	if (eih != 1) return false;
	if (!AlmostEqual(pose.translation().x(), 1.5) ||
	    !AlmostEqual(pose.translation().y(), -2.0) ||
	    !AlmostEqual(pose.translation().z(), 3.25)) {
		return false;
	}
	if (!pose.linear().isIdentity(1e-9)) return false;
	return true;
}

}  // namespace

int main(int argc, char* argv[]) {
	google::InitGoogleLogging(argv[0]);
	FLAGS_logtostderr = 1;

	LOG_ASSERT(TestRoundTrip()) << "In-memory JSON -> Affine3d round trip failed";
	LOG(INFO) << "In-memory JSON -> Eigen::Affine3d conversion OK";

	if (argc < 2) {
		LOG(INFO) << "Usage: ./load_handeye path/to/HandOnEyeCalib.json";
		return 0;
	}

	const std::string path = argv[1];
	poser::HandEyeCalib calib;
	LOG_ASSERT(LoadHandEyeCalibFromJsonFile(path, calib))
		<< "Failed to load hand-eye JSON: " << path;

	LOG(INFO) << "EIH = " << calib.eih;
	LOG(INFO) << "Affine3d translation = " << calib.pose.translation().transpose();
	LOG(INFO) << "Affine3d linear =\n" << calib.pose.linear();
	std::cout << calib.pose.matrix() << std::endl;
	return 0;
}
