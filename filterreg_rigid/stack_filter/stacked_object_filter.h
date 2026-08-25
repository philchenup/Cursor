#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>

#include <string>
#include <vector>

namespace poser {

// Grasp-time stacking filter: keep instances that are not pressed by another
// registered object. Input is a template cloud (model frame) plus a rigid pose
// per detection.
//
// Defaults match eye-in-hand / overhead camera bin picking:
//   - coordinates in millimetres
//   - camera +Z points into the scene (down), so "up"/graspable is -Z
//     (smaller camera Z = closer to the camera = on top)

enum class StackFilterMethod {
	// Orthographic occupancy in the plane perpendicular to up_axis.
	// overlap_ratio = area(cells with a closer/higher object) / area(this object).
	Projection2D = 0,

	// Fill each pose-aligned AABB with voxels, then the same coverage test.
	BoundingBox3D = 1,

	// Projection test plus a global height gate (TransferTech PointsUpJudge).
	HighestLayer = 2,
};

enum class LengthUnit {
	Millimeters = 0,
	Meters = 1,
};

struct StackFilterParams {
	StackFilterMethod method = StackFilterMethod::Projection2D;
	LengthUnit unit = LengthUnit::Millimeters;

	// Camera looks along +Z into the scene. When true, up_axis becomes -Z
	// unless the caller already set a non-+Z up_axis.
	bool camera_z_down = true;

	float overlap_ratio_threshold = 0.30f;

	// Orthographic cell size in the same unit as the cloud. <= 0 = auto
	// (2% of mean XY diagonal; ~2.5 mm for a ~120 mm flange).
	float pixel_size = 0.0f;

	float voxel_size = 0.0f;

	// A cell is pressed only if another instance is closer/higher by more
	// than this. <= 0 = auto (3 mm or 0.003 m).
	float height_margin = 0.0f;

	// Direction of "on top" / toward the gripper. Default -Z (camera down).
	Eigen::Vector3f up_axis = Eigen::Vector3f(0.0f, 0.0f, -1.0f);

	float layer_thickness = 0.0f;
	bool downsample = true;

	// Dilate each instance's 2D occupancy by this many cells so ring holes
	// and sparse templates still overlap stably. 1 is enough for flanges.
	int dilation_radius = 1;

	float expand_x = 1.0f;
	float expand_y = 1.0f;
	float expand_z = 3.0f;

	void Normalize();
	Eigen::Vector3f ResolvedUpAxis() const;
	float MillimetreScale() const;
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
	float mean_height = 0.0f;  // along up_axis; larger = closer to gripper
	float mean_z = 0.0f;       // raw camera/world Z of the centroid
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

	static pcl::PointCloud<pcl::PointXYZ>::Ptr TransformModel(
		const pcl::PointCloud<pcl::PointXYZ>& model,
		const Eigen::Matrix4f& pose);

	static const char* MethodName(StackFilterMethod method);
	static const char* UnitName(LengthUnit unit);

private:
	struct PreparedInstance {
		int index = -1;
		pcl::PointCloud<pcl::PointXYZ>::Ptr world_cloud;
		Eigen::Vector3f min_pt = Eigen::Vector3f::Zero();
		Eigen::Vector3f max_pt = Eigen::Vector3f::Zero();
		float mean_height = 0.0f;
		float mean_z = 0.0f;
	};

	std::vector<PreparedInstance> Prepare(
		const std::vector<RegisteredInstance>& instances) const;

	StackFilterOutput ScoreProjection2D(
		const std::vector<RegisteredInstance>& instances,
		const std::vector<PreparedInstance>& prepared,
		float cell_size,
		float height_margin) const;

	StackFilterOutput ScoreBoundingBox3D(
		const std::vector<RegisteredInstance>& instances,
		const std::vector<PreparedInstance>& prepared,
		float height_margin) const;

	StackFilterOutput ScoreHighestLayer(
		const std::vector<RegisteredInstance>& instances,
		const std::vector<PreparedInstance>& prepared,
		float cell_size,
		float height_margin) const;

	void FinalizeKeepFlags(StackFilterOutput* output) const;

	float ResolveCellSize(
		const std::vector<PreparedInstance>& prepared,
		float requested) const;

	float ResolveHeightMargin() const;

	StackFilterParams params_;
};

bool ParseStackFilterMethod(const std::string& name, StackFilterMethod* method);
bool ParseLengthUnit(const std::string& name, LengthUnit* unit);

}  // namespace poser
