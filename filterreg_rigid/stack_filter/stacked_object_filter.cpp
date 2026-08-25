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
		*axis_n = -Eigen::Vector3f::UnitZ();
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
	if (!enable || leaf <= 0.0f || !cloud || cloud->empty()) {
		auto copy = pcl::PointCloud<pcl::PointXYZ>::Ptr(
			new pcl::PointCloud<pcl::PointXYZ>(cloud ? *cloud : pcl::PointCloud<pcl::PointXYZ>()));
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

HeightMap DilateHeightMap(const HeightMap& src, int radius) {
	if (radius <= 0 || src.empty()) {
		return src;
	}
	HeightMap out = src;
	out.reserve(src.size() * static_cast<std::size_t>((2 * radius + 1) * (2 * radius + 1)));
	for (const auto& cell : src) {
		for (int dy = -radius; dy <= radius; ++dy) {
			for (int dx = -radius; dx <= radius; ++dx) {
				if (dx == 0 && dy == 0) {
					continue;
				}
				const CellKey key{cell.first.x + dx, cell.first.y + dy};
				auto it = out.find(key);
				if (it == out.end()) {
					out.emplace(key, cell.second);
				} else if (cell.second > it->second) {
					it->second = cell.second;
				}
			}
		}
	}
	return out;
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
	half = half.cwiseMax(Eigen::Vector3f::Constant(0.5f * voxel_size));

	const Eigen::Vector3f min_local = center - half;
	const Eigen::Vector3f max_local = center + half;
	const float leaf = std::max(voxel_size, 1e-3f);

	auto count_axis = [&](float lo, float hi) {
		const int n = std::max(1, static_cast<int>(std::ceil((hi - lo) / leaf)));
		return std::min(n, 80);
	};
	const int nx = count_axis(min_local.x(), max_local.x());
	const int ny = count_axis(min_local.y(), max_local.y());
	const int nz = count_axis(min_local.z(), max_local.z());
	const float dx = (max_local.x() - min_local.x()) / static_cast<float>(nx);
	const float dy = (max_local.y() - min_local.y()) / static_cast<float>(ny);
	const float dz = (max_local.z() - min_local.z()) / static_cast<float>(nz);

	filled->points.reserve(static_cast<std::size_t>(nx) * ny * nz);
	for (int ix = 0; ix < nx; ++ix) {
		const float x = min_local.x() + (ix + 0.5f) * dx;
		for (int iy = 0; iy < ny; ++iy) {
			const float y = min_local.y() + (iy + 0.5f) * dy;
			for (int iz = 0; iz < nz; ++iz) {
				const float z = min_local.z() + (iz + 0.5f) * dz;
				const Eigen::Vector4f world = pose * Eigen::Vector4f(x, y, z, 1.0f);
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

}  // namespace

void StackFilterParams::Normalize() {
	if (up_axis.squaredNorm() < 1e-12f) {
		up_axis = camera_z_down
			? Eigen::Vector3f(0.0f, 0.0f, -1.0f)
			: Eigen::Vector3f(0.0f, 0.0f, 1.0f);
	} else {
		up_axis.normalize();
	}
	// Overhead / in-hand camera: +Z into the scene, so "on top" is -Z.
	if (camera_z_down && up_axis.z() > 0.5f
		&& std::abs(up_axis.x()) < 0.2f && std::abs(up_axis.y()) < 0.2f) {
		up_axis = -up_axis;
	}
}

Eigen::Vector3f StackFilterParams::ResolvedUpAxis() const {
	StackFilterParams copy = *this;
	copy.Normalize();
	return copy.up_axis;
}

float StackFilterParams::MillimetreScale() const {
	return (unit == LengthUnit::Millimeters) ? 1.0f : 1000.0f;
}

StackedObjectFilter::StackedObjectFilter(StackFilterParams params)
	: params_(std::move(params)) {
	params_.Normalize();
}

void StackedObjectFilter::set_params(const StackFilterParams& params) {
	params_ = params;
	params_.Normalize();
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

const char* StackedObjectFilter::UnitName(LengthUnit unit) {
	return (unit == LengthUnit::Meters) ? "m" : "mm";
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

bool ParseLengthUnit(const std::string& name, LengthUnit* unit) {
	if (!unit) {
		return false;
	}
	if (name == "mm" || name == "millimeter" || name == "millimetre") {
		*unit = LengthUnit::Millimeters;
		return true;
	}
	if (name == "m" || name == "meter" || name == "metre") {
		*unit = LengthUnit::Meters;
		return true;
	}
	return false;
}

std::vector<StackedObjectFilter::PreparedInstance> StackedObjectFilter::Prepare(
	const std::vector<RegisteredInstance>& instances
) const {
	std::vector<PreparedInstance> prepared;
	prepared.reserve(instances.size());
	const Eigen::Vector3f axis_n = params_.ResolvedUpAxis();

	for (std::size_t i = 0; i < instances.size(); ++i) {
		PreparedInstance item;
		item.index = static_cast<int>(i);
		if (!instances[i].model || instances[i].model->empty()) {
			item.world_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
			prepared.push_back(item);
			continue;
		}
		item.world_cloud = TransformModel(*instances[i].model, instances[i].pose);

		Eigen::Vector4f min_pt, max_pt, centroid;
		pcl::getMinMax3D(*item.world_cloud, min_pt, max_pt);
		pcl::compute3DCentroid(*item.world_cloud, centroid);
		item.min_pt = min_pt.head<3>();
		item.max_pt = max_pt.head<3>();
		item.mean_z = centroid.z();
		item.mean_height = Eigen::Vector3f(centroid.x(), centroid.y(), centroid.z()).dot(axis_n);
		prepared.push_back(item);
	}
	return prepared;
}

float StackedObjectFilter::ResolveCellSize(
	const std::vector<PreparedInstance>& prepared,
	float requested
) const {
	const float mm = params_.MillimetreScale();
	const float min_cell = 0.2f * mm;       // 0.2 mm
	const float fallback = 2.5f * mm;       // 2.5 mm
	if (requested > 0.0f) {
		return std::max(requested, min_cell);
	}
	double sum_xy = 0.0;
	int count = 0;
	for (const auto& item : prepared) {
		if (!item.world_cloud || item.world_cloud->empty()) {
			continue;
		}
		const float dx = item.max_pt.x() - item.min_pt.x();
		const float dy = item.max_pt.y() - item.min_pt.y();
		sum_xy += std::sqrt(dx * dx + dy * dy);
		++count;
	}
	if (count == 0) {
		return fallback;
	}
	// Mech-Vision: ~2% of the XY diagonal of the template.
	return std::max(min_cell, static_cast<float>(0.02 * (sum_xy / count)));
}

float StackedObjectFilter::ResolveHeightMargin() const {
	if (params_.height_margin > 0.0f) {
		return params_.height_margin;
	}
	return 3.0f * params_.MillimetreScale();
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
	float cell_size,
	float height_margin
) const {
	StackFilterOutput output;
	output.instances.resize(instances.size());

	Eigen::Vector3f axis_u, axis_v, axis_n;
	BuildTangentFrame(params_.ResolvedUpAxis(), &axis_u, &axis_v, &axis_n);

	std::vector<HeightMap> maps(prepared.size());
	HeightMap global_max;
	for (std::size_t i = 0; i < prepared.size(); ++i) {
		if (!prepared[i].world_cloud) {
			continue;
		}
		auto raw = BuildHeightMap(
			*prepared[i].world_cloud, axis_u, axis_v, axis_n, cell_size);
		maps[i] = DilateHeightMap(raw, params_.dilation_radius);
		for (const auto& cell : maps[i]) {
			auto it = global_max.find(cell.first);
			if (it == global_max.end()) {
				global_max.emplace(cell.first, cell.second);
			} else if (cell.second > it->second) {
				it->second = cell.second;
			}
		}
	}

	for (std::size_t i = 0; i < instances.size(); ++i) {
		InstanceFilterResult result;
		result.index = static_cast<int>(i);
		result.id = instances[i].id;
		result.score = instances[i].score;
		result.pose = instances[i].pose;
		result.mean_height = prepared[i].mean_height;
		result.mean_z = prepared[i].mean_z;
		result.occupied_cells = static_cast<int>(maps[i].size());

		int pressed = 0;
		for (const auto& cell : maps[i]) {
			auto it = global_max.find(cell.first);
			if (it != global_max.end() && it->second > cell.second + height_margin) {
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
	const std::vector<PreparedInstance>& prepared,
	float height_margin
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

	return ScoreProjection2D(solid, solid_prep, voxel, height_margin);
}

StackFilterOutput StackedObjectFilter::ScoreHighestLayer(
	const std::vector<RegisteredInstance>& instances,
	const std::vector<PreparedInstance>& prepared,
	float cell_size,
	float height_margin
) const {
	auto output = ScoreProjection2D(instances, prepared, cell_size, height_margin);

	float max_height = -std::numeric_limits<float>::infinity();
	std::vector<float> extents;
	extents.reserve(prepared.size());
	const Eigen::Vector3f axis_n = params_.ResolvedUpAxis();
	for (const auto& item : prepared) {
		max_height = std::max(max_height, item.mean_height);
		const float extent = std::abs((item.max_pt - item.min_pt).dot(axis_n.cwiseAbs()));
		if (extent > 0.0f) {
			extents.push_back(std::abs(item.max_pt.z() - item.min_pt.z()));
		}
	}

	const float mm = params_.MillimetreScale();
	float thickness = params_.layer_thickness;
	if (thickness <= 0.0f) {
		if (extents.empty()) {
			thickness = 20.0f * mm;
		} else {
			std::nth_element(
				extents.begin(),
				extents.begin() + static_cast<std::ptrdiff_t>(extents.size() / 2),
				extents.end());
			thickness = std::max(extents[extents.size() / 2], 1.0f * mm);
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
	auto prepared = Prepare(instances);
	const float cell = ResolveCellSize(prepared, params_.pixel_size);
	const float margin = ResolveHeightMargin();
	if (params_.downsample) {
		for (auto& item : prepared) {
			item.world_cloud = MaybeDownsample(item.world_cloud, cell, true);
		}
	}
	switch (params_.method) {
	case StackFilterMethod::BoundingBox3D:
		return ScoreBoundingBox3D(instances, prepared, margin);
	case StackFilterMethod::HighestLayer:
		return ScoreHighestLayer(instances, prepared, cell, margin);
	case StackFilterMethod::Projection2D:
	default:
		return ScoreProjection2D(instances, prepared, cell, margin);
	}
}

}  // namespace poser
