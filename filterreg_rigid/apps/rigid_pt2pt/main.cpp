//
// Created by wei on 2/28/19.
// Adapted for PCL-free standalone package.
//

#include "corr_search/gmm/gmm.h"
#include "io/pcd_io.h"
#include "kinematic/rigid/rigid.h"
#include "visualizer/debug_visualizer.h"

#include <chrono>
#include <cmath>
#include <string>

void load_test_data(
	const std::string& cloud_path,
	poser::FeatureMap& geometric_model,
	poser::FeatureMap& observation,
	poser::RigidKinematicModel& kinematic
) {
	using namespace poser;

	std::vector<PcdPointXYZ> points;
	LOG_ASSERT(LoadPcdXYZ(cloud_path, points)) << "Failed to load PCD: " << cloud_path;
	LOG(INFO) << "The size of the point cloud is " << points.size();

	LoadVerticesToFeatureMap(
		geometric_model,
		CommonFeatureChannelKey::ReferenceVertex(),
		points);
	LoadVerticesToFeatureMap(
		observation,
		CommonFeatureChannelKey::ObservationVertexCamera(),
		points);

	// Random initial alignment
	const auto angle_error = 0.87f; // About 50 degree
	Eigen::Vector3f axis;
	axis.setRandom();
	axis.normalize();
	LOG(INFO) << "The initial rotation error is " << angle_error * 180.0 / 3.24
	          << " degree w.r.t a random axis.";

	const auto translation_error = 0.1f;
	Eigen::Isometry3f eigen_rand_init_SE3(Eigen::AngleAxisf(angle_error, axis));
	axis.setRandom();
	axis.normalize();
	eigen_rand_init_SE3.translation() = translation_error * axis;
	mat34 rand_init_SE3(eigen_rand_init_SE3);
	LOG(INFO) << "The initial translation error is " << translation_error
	          << " meter w.r.t a random direction.";

	kinematic.SetMotionParameter(rand_init_SE3);
	kinematic.CheckGeometricModelAndAllocateAttribute(geometric_model);
}

float compute_meansquare_error(
	const poser::FeatureMap& model,
	const poser::mat34& estimated_pose,
	const poser::mat34& gt_pose
) {
	using namespace poser;
	auto reference_vertex_channel = CommonFeatureChannelKey::ReferenceVertex();
	const auto ref_vertex = model.GetTypedFeatureValueReadOnly<float4>(
		reference_vertex_channel, MemoryContext::CpuMemory);

	float accumlate_mse = 0.0f;
	float3 gt_vertex, estimated_vertex;
	for (auto i = 0; i < ref_vertex.Size(); i++) {
		const auto vertex_i = ref_vertex[i];
		estimated_vertex = estimated_pose.rotation() * vertex_i + estimated_pose.translation;
		gt_vertex = gt_pose.rotation() * vertex_i + gt_pose.translation;
		accumlate_mse += norm(estimated_vertex - gt_vertex);
	}
	return (accumlate_mse / ref_vertex.Size());
}

void process_pt2pt(const std::string& cloud_path) {
	using namespace poser;
	FeatureMap model, observation;
	RigidKinematicModel kinematic(MemoryContext::CpuMemory);
	load_test_data(cloud_path, model, observation, kinematic);

	// If you want fixed sigma, use GMMPermutohedralFixedSigma<3> instead.
	GMMPermutohedralUpdatedSigma corr_search(
		CommonFeatureChannelKey::ObservationVertexCamera(),
		CommonFeatureChannelKey::LiveVertex());

	DenseGeometricTarget target;
	corr_search.CheckAndAllocateTarget(observation, model, target);

	RigidPoint2PointTermAssemblerCPU point2point_assembler;
	point2point_assembler.CheckAndAllocate(model, kinematic, target);

	RigidPoint2PointKabsch kabsch;
	kabsch.CheckAndAllocate(model, kinematic, target);

	using namespace std::chrono;
	auto t1 = high_resolution_clock::now();

	const float init_guassian_sigma = 0.03f;
	const float min_gaussian_sigma = 0.002f;
	corr_search.UpdateObservation(observation, init_guassian_sigma);
	for (auto i = 0; i < 16; i++) {
		UpdateLiveVertexCPU(kinematic, model);

		if (i >= 1) {
			auto sigma = corr_search.ComputeSigmaValue(model, target);
			if (!std::isnan(sigma) && sigma > min_gaussian_sigma)
				corr_search.UpdateObservation(observation, sigma);
		}

		corr_search.ComputeTarget(observation, model, target);

		mat34 transform;
		kabsch.ComputeTransformToTarget(model, kinematic, target, transform);
		kinematic.SetMotionParameter(transform);
	}

	auto t2 = high_resolution_clock::now();
	LOG(INFO) << "The initial sigma value is " << init_guassian_sigma << " meter.";
	LOG(INFO) << "The running time is "
	          << duration_cast<milliseconds>(t2 - t1).count() << " milliseconds.";
	LOG(INFO) << "The final averaged alignment error per point is "
	          << compute_meansquare_error(model, kinematic.GetRigidTransform(), mat34::identity())
	          << " m.";

	UpdateLiveVertexCPU(kinematic, model);
	auto live_vertex = model.GetTypedFeatureValueReadOnly<float4>(
		CommonFeatureChannelKey::LiveVertex(), MemoryContext::CpuMemory);
	auto depth_vertex = observation.GetTypedFeatureValueReadOnly<float4>(
		CommonFeatureChannelKey::ObservationVertexCamera(), MemoryContext::CpuMemory);
	DebugVisualizer::DrawMatchedCloudPair(live_vertex, depth_vertex);
}

int main(int argc, char* argv[]) {
	google::InitGoogleLogging(argv[0]);
	FLAGS_logtostderr = 1;
	LOG_ASSERT(argc == 2) << "Usage: ./rigid_pt2pt path/to/bunny.pcd";
	process_pt2pt(argv[1]);
	return 0;
}
