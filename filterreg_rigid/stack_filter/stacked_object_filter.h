#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>

#include <string>
#include <vector>

namespace poser {

// Grasp-time stacking filter: keep instances that are not pressed down by
// another registered object. Input is a template cloud (model frame) plus a
// rigid pose per detection — the same contract used by Mech-Vision
// "Remove Overlapped Objects" and TransferTech "Extract Top-layer Point Clouds".
//
// Units are meters. Default "up" is +Z.

enum class StackFilterMethod {
	// Orthographic 2D occupancy along the up-axis.
	// overlap_ratio = area(cells with a higher object) / area(this object).
	// Recommended after surface-template 3D matching (Mech-Vision Projection 2D).
	Projection2D = 0,

	// Fill each pose-aligned AABB with voxels and apply the same "something
	// above" test. Better for sparse/edge templates (Mech-Vision Bounding Box 3D).
	BoundingBox3D = 1,

	// Global height gate: keep objects whose mean height is within
	// layer_thickness/2 of the highest instance (TransferTech PointsUpJudge).
	HighestLayer = 2,
};

struct StackFilterParams {
	StackFilterMethod method = StackFilterMethod::Projection2D;

	// Drop an instance when overlap_ratio exceeds this value. Default 0.30
	// matches Mech-Vision / TransferTech.
	float overlap_ratio_threshold = 0.30f;

	// Orthographic cell size (m). <= 0 means auto: 2% of the mean cloud diagonal.
	float pixel_size = 0.0f;

	// Voxel size for BoundingBox3D (m). <= 0 means auto (same rule as pixel_size).
	float voxel_size = 0.0f;

	// A column counts as pressed only if another instance is higher by this
	// margin. Absorbs contact between stacked faces and pose noise.
	float height_margin = 0.003f;

	// Gravity / "up" direction in the pose frame (usually the robot/world Z).
	Eigen::Vector3f up_axis = Eigen::Vector3f::UnitZ();

	// HighestLayer only. <= 0 means auto: median instance height extent.
	float layer_thickness = 0.0f;

	// Downsample transformed clouds with pcl::VoxelGrid before scoring.
	bool downsample = true;

	// AABB expansion along the pose axes (BoundingBox3D). Z defaults larger
	// so a thin surface cloud still has volume, matching Mech-Vision.
	float expand_x = 1.0f;
	float expand_y = 1.0f;
	float expand_z = 3.0f;
};

struct RegisteredInstance {
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	std::string id;
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr model;
	Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
	float score = 1.0f;
};

struct InstanceFilterResult {
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	int index = -1;
	std::string id;
	float score = 1.0f;
	float overlap_ratio = 0.0f;
	float mean_height = 0.0f;
	int occupied_cells = 0;
	int pressed_cells = 0;
	bool kept = true;
	Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
};

struct StackFilterOutput {
	std::vector<InstanceFilterResult> instances;
	std::vector<int> kept_indices;  // sorted by mean_height descending
};

class StackedObjectFilter {
public:
	explicit StackedObjectFilter(StackFilterParams params = StackFilterParams());

	void set_params(const StackFilterParams& params);
	const StackFilterParams& params() const { return params_; }

	StackFilterOutput Filter(const std::vector<RegisteredInstance>& instances) const;

	// Transform a model-frame template by a 4x4 pose (PCL transformPointCloud).
	static pcl::PointCloud<pcl::PointXYZ>::Ptr TransformModel(
		const pcl::PointCloud<pcl::PointXYZ>& model,
		const Eigen::Matrix4f& pose);

	static const char* MethodName(StackFilterMethod method);

private:
	struct PreparedInstance {
		int index = -1;
		pcl::PointCloud<pcl::PointXYZ>::Ptr world_cloud;
		Eigen::Vector3f min_pt = Eigen::Vector3f::Zero();
		Eigen::Vector3f max_pt = Eigen::Vector3f::Zero();
		float mean_height = 0.0f;
	};

	std::vector<PreparedInstance> Prepare(
		const std::vector<RegisteredInstance>& instances) const;

	StackFilterOutput ScoreProjection2D(
		const std::vector<RegisteredInstance>& instances,
		const std::vector<PreparedInstance>& prepared,
		float cell_size) const;

	StackFilterOutput ScoreBoundingBox3D(
		const std::vector<RegisteredInstance>& instances,
		const std::vector<PreparedInstance>& prepared) const;

	StackFilterOutput ScoreHighestLayer(
		const std::vector<RegisteredInstance>& instances,
		const std::vector<PreparedInstance>& prepared) const;

	void FinalizeKeepFlags(StackFilterOutput* output) const;

	float ResolveCellSize(
		const std::vector<PreparedInstance>& prepared,
		float requested) const;

	StackFilterParams params_;
};

// Parse CLI / JSON method names: projection_2d, bounding_box_3d, highest_layer.
bool ParseStackFilterMethod(const std::string& name, StackFilterMethod* method);

}  // namespace poser
