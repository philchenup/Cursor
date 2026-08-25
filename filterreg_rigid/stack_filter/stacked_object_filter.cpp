#include "stack_filter/stacked_object_filter.h"

#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <utility>

namespace poser {
namespace {

struct CellKey {
	int x = 0;
	int y = 0;

	bool operator==(const CellKey& other) const {
		return x == other.x && y == other.y;
	}
};

struct CellKeyHash {
	std::size_t operator()(const CellKey& key) const {
		const std::uint64_t ux = static_cast<std::uint32_t>(key.x);
		const std::uint64_t uy = static_cast<std::uint32_t>(key.y);
		return static_cast<std::size_t>((ux << 32) ^ uy);
	}
};

using HeightMap = std::unordered_map<CellKey, float, CellKeyHash>;

void BuildTangentFrame(
	const Eigen::Vector3f& up,
	Eigen::Vector3f* axis_u,
	Eigen::Vector3f* axis_v,
	Eigen::Vector3f* axis_n
) {
	*axis_n = up;
	if (axis_n->squaredNorm() < 1e-12f) {
		*axis_n = Eigen::Vector3f::UnitZ();
	}
	axis_n->normalize();

	const Eigen::Vector3f tmp = (std::abs(axis_n->z()) < 0.9f)
		? Eigen::Vector3f::UnitZ()
		: Eigen::Vector3f::UnitX();
	*axis_u = axis_n->cross(tmp).normalized();
	*axis_v = axis_n->cross(*axis_u).normalized();
}

pcl::PointCloud<pcl::PointXYZ>::Ptr MaybeDownsample(
	const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
	float leaf,
	bool enable
) {
	if (!enable || leaf <= 0.0f || cloud->empty()) {
		auto copy = pcl::PointCloud<pcl::PointXYZ>::Ptr(
			new pcl::PointCloud<pcl::PointXYZ>(*cloud));
		return copy;
	}

	pcl::VoxelGrid<pcl::PointXYZ> voxel;
	voxel.setInputCloud(cloud);
	voxel.setLeafSize(leaf, leaf, leaf);
	auto filtered = pcl::PointCloud<pcl::PointXYZ>::Ptr(
		new pcl::PointCloud<pcl::PointXYZ>());
	voxel.filter(*filtered);
	if (filtered->empty()) {
		filtered->push_back(cloud->points.front());
	}
	return filtered;
}

HeightMap BuildHeightMap(
	const pcl::PointCloud<pcl::PointXYZ>& cloud,
	const Eigen::Vector3f& axis_u,
	const Eigen::Vector3f& axis_v,
	const Eigen::Vector3f& axis_n,
	float cell_size
) {
	HeightMap map;
	if (cloud.empty() || cell_size <= 0.0f) {
		return map;
	}
	const float inv = 1.0f / cell_size;
	map.reserve(cloud.size());
	for (const auto& pt : cloud.points) {
		if (!pcl::isFinite(pt)) {
			continue;
		}
		const Eigen::Vector3f p(pt.x, pt.y, pt.z);
		const CellKey key{
			static_cast<int>(std::floor(p.dot(axis_u) * inv)),
			static_cast<int>(std::floor(p.dot(axis_v) * inv))};
		const float height = p.dot(axis_n);
		auto it = map.find(key);
		if (it == map.end()) {
			map.emplace(key, height);
		} else if (height > it->second) {
			it->second = height;
		}
	}
	return map;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr FillPoseAlignedBox(
	const pcl::PointCloud<pcl::PointXYZ>& model,
	const Eigen::Matrix4f& pose,
	float voxel_size,
	float expand_x,
	float expand_y,
	float expand_z
) {
	auto filled = pcl::PointCloud<pcl::PointXYZ>::Ptr(
		new pcl::PointCloud<pcl::PointXYZ>());
	if (model.empty()) {
		return filled;
	}

	Eigen::Vector4f min_pt, max_pt;
	pcl::getMinMax3D(model, min_pt, max_pt);
	Eigen::Vector3f center = 0.5f * (min_pt.head<3>() + max_pt.head<3>());
	Eigen::Vector3f half = 0.5f * (max_pt.head<3>() - min_pt.head<3>());
	half.x() *= std::max(expand_x, 1e-3f);
	half.y() *= std::max(expand_y, 1e-3f);
	half.z() *= std::max(expand_z, 1e-3f);
	// Guard zero-thickness templates (planar face clouds).
	half = half.cwiseMax(Eigen::Vector3f::Constant(0.5f * voxel_size));

	const Eigen::Vector3f min_local = center - half;
	const Eigen::Vector3f max_local = center + half;
	const float leaf = std::max(voxel_size, 1e-4f);

	const int nx = std::max(1, static_cast<int>(std::ceil((max_local.x() - min_local.x()) / leaf)));
	const int ny = std::max(1, static_cast<int>(std::ceil((max_local.y() - min_local.y()) / leaf)));
	const int nz = std::max(1, static_cast<int>(std::ceil((max_local.z() - min_local.z()) / leaf)));

	filled->points.reserve(static_cast<std::size_t>(nx) * ny * nz);
	for (int ix = 0; ix < nx; ++ix) {
		const float x = min_local.x() + (ix + 0.5f) * leaf;
		for (int iy = 0; iy < ny; ++iy) {
			const float y = min_local.y() + (iy + 0.5f) * leaf;
			for (int iz = 0; iz < nz; ++iz) {
				const float z = min_local.z() + (iz + 0.5f) * leaf;
				const Eigen::Vector4f local(x, y, z, 1.0f);
				const Eigen::Vector4f world = pose * local;
				pcl::PointXYZ pt;
				pt.x = world.x();
				pt.y = world.y();
				pt.z = world.z();
				filled->points.push_back(pt);
			}
		}
	}
	filled->width = static_cast<std::uint32_t>(filled->points.size());
	filled->height = 1;
	filled->is_dense = true;
	return filled;
}

float MeanHeight(
	const pcl::PointCloud<pcl::PointXYZ>& cloud,
	const Eigen::Vector3f& axis_n
) {
	if (cloud.empty()) {
		return 0.0f;
	}
	Eigen::Vector4f centroid;
	pcl::compute3DCentroid(cloud, centroid);
	return Eigen::Vector3f(centroid.x(), centroid.y(), centroid.z()).dot(axis_n);
}

}  // namespace

StackedObjectFilter::StackedObjectFilter(StackFilterParams params)
	: params_(std::move(params)) {}

void StackedObjectFilter::set_params(const StackFilterParams& params) {
	params_ = params;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr StackedObjectFilter::TransformModel(
	const pcl::PointCloud<pcl::PointXYZ>& model,
	const Eigen::Matrix4f& pose
) {
	auto world = pcl::PointCloud<pcl::PointXYZ>::Ptr(
		new pcl::PointCloud<pcl::PointXYZ>());
	pcl::transformPointCloud(model, *world, pose);
	return world;
}

const char* StackedObjectFilter::MethodName(StackFilterMethod method) {
	switch (method) {
	case StackFilterMethod::Projection2D:
		return "projection_2d";
	case StackFilterMethod::BoundingBox3D:
		return "bounding_box_3d";
	case StackFilterMethod::HighestLayer:
		return "highest_layer";
	}
	return "unknown";
}

bool ParseStackFilterMethod(const std::string& name, StackFilterMethod* method) {
	if (!method) {
		return false;
	}
	if (name == "projection_2d" || name == "projection2d" || name == "2d") {
		*method = StackFilterMethod::Projection2D;
		return true;
	}
	if (name == "bounding_box_3d" || name == "bbox3d" || name == "3d") {
		*method = StackFilterMethod::BoundingBox3D;
		return true;
	}
	if (name == "highest_layer" || name == "top_layer" || name == "height") {
		*method = StackFilterMethod::HighestLayer;
		return true;
	}
	return false;
}

std::vector<StackedObjectFilter::PreparedInstance> StackedObjectFilter::Prepare(
	const std::vector<RegisteredInstance>& instances
) const {
	std::vector<PreparedInstance> prepared;
	prepared.reserve(instances.size());

	Eigen::Vector3f axis_u, axis_v, axis_n;
	BuildTangentFrame(params_.up_axis, &axis_u, &axis_v, &axis_n);

	for (std::size_t i = 0; i < instances.size(); ++i) {
		PreparedInstance item;
		item.index = static_cast<int>(i);
		if (!instances[i].model || instances[i].model->empty()) {
			item.world_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
			prepared.push_back(item);
			continue;
		}
		auto world = TransformModel(*instances[i].model, instances[i].pose);
		const float leaf = (params_.pixel_size > 0.0f) ? params_.pixel_size : 0.0f;
		item.world_cloud = MaybeDownsample(world, leaf, params_.downsample);

		Eigen::Vector4f min_pt, max_pt;
		pcl::getMinMax3D(*item.world_cloud, min_pt, max_pt);
		item.min_pt = min_pt.head<3>();
		item.max_pt = max_pt.head<3>();
		item.mean_height = MeanHeight(*item.world_cloud, axis_n);
		prepared.push_back(item);
	}
	return prepared;
}

float StackedObjectFilter::ResolveCellSize(
	const std::vector<PreparedInstance>& prepared,
	float requested
) const {
	if (requested > 0.0f) {
		return requested;
	}
	double sum_diag = 0.0;
	int count = 0;
	for (const auto& item : prepared) {
		if (!item.world_cloud || item.world_cloud->empty()) {
			continue;
		}
		const Eigen::Vector3f extent = item.max_pt - item.min_pt;
		sum_diag += extent.norm();
		++count;
	}
	if (count == 0) {
		return 0.005f;
	}
	// Mech-Vision default: cube side = 2% of the point-cloud diagonal.
	return static_cast<float>(std::max(1e-4, 0.02 * (sum_diag / count)));
}

void StackedObjectFilter::FinalizeKeepFlags(StackFilterOutput* output) const {
	output->kept_indices.clear();
	for (auto& inst : output->instances) {
		inst.kept = inst.overlap_ratio <= params_.overlap_ratio_threshold;
		if (inst.kept) {
			output->kept_indices.push_back(inst.index);
		}
	}
	std::sort(
		output->kept_indices.begin(),
		output->kept_indices.end(),
		[&](int a, int b) {
			return output->instances[a].mean_height > output->instances[b].mean_height;
		});
}

StackFilterOutput StackedObjectFilter::ScoreProjection2D(
	const std::vector<RegisteredInstance>& instances,
	const std::vector<PreparedInstance>& prepared,
	float cell_size
) const {
	StackFilterOutput output;
	output.instances.resize(instances.size());

	Eigen::Vector3f axis_u, axis_v, axis_n;
	BuildTangentFrame(params_.up_axis, &axis_u, &axis_v, &axis_n);

	std::vector<HeightMap> maps(prepared.size());
	for (std::size_t i = 0; i < prepared.size(); ++i) {
		if (prepared[i].world_cloud) {
			maps[i] = BuildHeightMap(
				*prepared[i].world_cloud, axis_u, axis_v, axis_n, cell_size);
		}
	}

	for (std::size_t i = 0; i < instances.size(); ++i) {
		InstanceFilterResult result;
		result.index = static_cast<int>(i);
		result.id = instances[i].id;
		result.score = instances[i].score;
		result.pose = instances[i].pose;
		result.mean_height = prepared[i].mean_height;
		result.occupied_cells = static_cast<int>(maps[i].size());

		int pressed = 0;
		for (const auto& cell : maps[i]) {
			bool covered = false;
			for (std::size_t j = 0; j < maps.size() && !covered; ++j) {
				if (j == i) {
					continue;
				}
				auto it = maps[j].find(cell.first);
				if (it == maps[j].end()) {
					continue;
				}
				if (it->second > cell.second + params_.height_margin) {
					covered = true;
				}
			}
			if (covered) {
				++pressed;
			}
		}
		result.pressed_cells = pressed;
		result.overlap_ratio = (result.occupied_cells > 0)
			? static_cast<float>(pressed) / static_cast<float>(result.occupied_cells)
			: 0.0f;
		output.instances[i] = result;
	}

	FinalizeKeepFlags(&output);
	return output;
}

StackFilterOutput StackedObjectFilter::ScoreBoundingBox3D(
	const std::vector<RegisteredInstance>& instances,
	const std::vector<PreparedInstance>& prepared
) const {
	const float voxel = ResolveCellSize(
		prepared,
		params_.voxel_size > 0.0f ? params_.voxel_size : params_.pixel_size);

	std::vector<RegisteredInstance> solid = instances;
	std::vector<PreparedInstance> solid_prep = prepared;
	for (std::size_t i = 0; i < instances.size(); ++i) {
		if (!instances[i].model) {
			continue;
		}
		auto filled = FillPoseAlignedBox(
			*instances[i].model,
			instances[i].pose,
			voxel,
			params_.expand_x,
			params_.expand_y,
			params_.expand_z);
		solid_prep[i].world_cloud = filled;
		if (!filled->empty()) {
			Eigen::Vector4f min_pt, max_pt;
			pcl::getMinMax3D(*filled, min_pt, max_pt);
			solid_prep[i].min_pt = min_pt.head<3>();
			solid_prep[i].max_pt = max_pt.head<3>();
		}
	}

	return ScoreProjection2D(solid, solid_prep, voxel);
}

StackFilterOutput StackedObjectFilter::ScoreHighestLayer(
	const std::vector<RegisteredInstance>& instances,
	const std::vector<PreparedInstance>& prepared
) const {
	const float cell_size = ResolveCellSize(prepared, params_.pixel_size);
	auto output = ScoreProjection2D(instances, prepared, cell_size);

	float max_height = -std::numeric_limits<float>::infinity();
	std::vector<float> extents;
	extents.reserve(prepared.size());
	for (const auto& item : prepared) {
		max_height = std::max(max_height, item.mean_height);
		const float extent = (item.max_pt - item.min_pt).norm();
		if (extent > 0.0f) {
			extents.push_back(item.max_pt.z() - item.min_pt.z());
		}
	}

	float thickness = params_.layer_thickness;
	if (thickness <= 0.0f) {
		if (extents.empty()) {
			thickness = 0.05f;
		} else {
			std::nth_element(
				extents.begin(),
				extents.begin() + static_cast<std::ptrdiff_t>(extents.size() / 2),
				extents.end());
			thickness = std::max(extents[extents.size() / 2], 1e-3f);
		}
	}

	const float gate = max_height - 0.5f * thickness;
	output.kept_indices.clear();
	for (std::size_t i = 0; i < output.instances.size(); ++i) {
		const bool in_top_layer = output.instances[i].mean_height >= gate;
		output.instances[i].kept = in_top_layer &&
			output.instances[i].overlap_ratio <= params_.overlap_ratio_threshold;
		if (output.instances[i].kept) {
			output.kept_indices.push_back(output.instances[i].index);
		}
	}
	std::sort(
		output.kept_indices.begin(),
		output.kept_indices.end(),
		[&](int a, int b) {
			return output.instances[a].mean_height > output.instances[b].mean_height;
		});
	return output;
}

StackFilterOutput StackedObjectFilter::Filter(
	const std::vector<RegisteredInstance>& instances
) const {
	const auto prepared = Prepare(instances);
	switch (params_.method) {
	case StackFilterMethod::BoundingBox3D:
		return ScoreBoundingBox3D(instances, prepared);
	case StackFilterMethod::HighestLayer:
		return ScoreHighestLayer(instances, prepared);
	case StackFilterMethod::Projection2D:
	default:
		return ScoreProjection2D(
			instances,
			prepared,
			ResolveCellSize(prepared, params_.pixel_size));
	}
}

}  // namespace poser
