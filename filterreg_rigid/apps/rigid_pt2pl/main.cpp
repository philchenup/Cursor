//
// Created by wei on 2/28/19.
// Adapted for PCL-free standalone package.
//

#include "corr_search/gmm/gmm.h"
#include "io/pcd_io.h"
#include "kinematic/rigid/rigid.h"
#include "visualizer/debug_visualizer.h"

#include <glog/logging.h>

#include <chrono>
#include <cmath>
#include <string>

void load_feature_map(poser::FeatureMap& feature_map, const std::string& pcd_cloud_path) {
	using namespace poser;
	std::vector<PcdPointXYZNormal> points;
	LOG_ASSERT(LoadPcdXYZNormal(pcd_cloud_path, points))
		<< "Failed to load PCD: " << pcd_cloud_path;
	LoadVerticesAndNormalsToFeatureMap(
		feature_map,
		CommonFeatureChannelKey::ObservationVertexCamera(),
		CommonFeatureChannelKey::ObservationNormalCamera(),
		points);
}

void process_pt2pl(const std::string& model_path, const std::string& obs_path) {
	using namespace poser;
	FeatureMap observation, model;
	load_feature_map(model, model_path);
	load_feature_map(observation, obs_path);
	LOG(INFO) << "The number of points in model is " << model.GetDenseFeatureDim().total_size();
	LOG(INFO) << "The number of points in observation is "
	          << observation.GetDenseFeatureDim().total_size();

	RigidKinematicModel kinematic(
		MemoryContext::CpuMemory,
		CommonFeatureChannelKey::ObservationVertexCamera());
	kinematic.CheckGeometricModelAndAllocateAttribute(model);
	kinematic.SetMotionParameter(mat34::identity());

	GMMPermutohedralFixedSigmaPt2Pl<3> corr_search(
		CommonFeatureChannelKey::ObservationVertexCamera(),
		CommonFeatureChannelKey::ObservationNormalCamera(),
		CommonFeatureChannelKey::LiveVertex());

	DenseGeometricTarget target;
	corr_search.CheckAndAllocateTarget(observation, model, target);

	Eigen::Matrix6f JtJ_point2plane, JtJ_point2point, JtJ;
	Eigen::Vector6f Jte_point2plane, Jte_point2point, Jte, d_twist;
	RigidPoint2PlaneTermAssemblerCPU point2plane;
	RigidPoint2PointTermAssemblerCPU point2point;
	point2plane.CheckAndAllocate(model, kinematic, target);
	point2point.CheckAndAllocate(model, kinematic, target);

	using namespace std::chrono;
	auto t1 = high_resolution_clock::now();

	UpdateLiveVertexCPU(kinematic, model);
	// Coarse-to-fine GMM bandwidth: a fixed 8 cm kernel mixes parallel
	// surfaces and leaves a systematic gap. Anneal toward point spacing
	// so the last iterations snap onto a single plane.
	float gaussian_sigma = 0.08f;
	const float min_gaussian_sigma = 0.004f;
	for (auto i = 0; i < 20; i++) {
		corr_search.UpdateObservation(observation, gaussian_sigma);
		corr_search.ComputeTarget(observation, model, target);

		JtJ.setZero();
		Jte.setZero();
		JtJ_point2plane.setZero();
		Jte_point2plane.setZero();
		JtJ_point2point.setZero();
		Jte_point2point.setZero();

		point2plane.ProcessAssemble(model, kinematic, target, JtJ_point2plane, Jte_point2plane);

		JtJ = JtJ_point2plane + JtJ_point2point;
		Jte = Jte_point2plane + Jte_point2point;
		d_twist = JtJ.ldlt().solve(Jte);

		kinematic.UpdateWithTwist(d_twist);
		UpdateLiveVertexCPU(kinematic, model);

		gaussian_sigma = std::max(min_gaussian_sigma, gaussian_sigma * 0.75f);
	}
	LOG(INFO) << "Final GMM sigma is " << gaussian_sigma << " meter.";

	auto t2 = high_resolution_clock::now();
	LOG(INFO) << "The time is " << duration_cast<milliseconds>(t2 - t1).count();

	auto live_vertex = model.GetTypedFeatureValueReadOnly<float4>(
		CommonFeatureChannelKey::LiveVertex(), MemoryContext::CpuMemory);
	auto depth_vertex = observation.GetTypedFeatureValueReadOnly<float4>(
		CommonFeatureChannelKey::ObservationVertexCamera(), MemoryContext::CpuMemory);
	DebugVisualizer::DrawMatchedCloudPair(live_vertex, depth_vertex);
}

int main(int argc, char* argv[]) {
	google::InitGoogleLogging(argv[0]);
	FLAGS_logtostderr = 1;
	LOG_ASSERT(argc == 3) << "Usage: ./rigid_pt2pl /path/to/cloud_0.pcd /path/to/cloud_1.pcd";
	process_pt2pl(argv[1], argv[2]);
	return 0;
}
