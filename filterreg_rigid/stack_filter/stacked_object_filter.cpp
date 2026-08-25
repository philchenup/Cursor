#include "stack_filter/stacked_object_filter.h"

#include <pcl/common/centroid.h>
#include <pcl/common/common.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <vector>

namespace poser {
namespace {

struct Cell {
	int x = 0;
	int y = 0;
	bool operator==(const Cell& o) const { return x == o.x && y == o.y; }
};

struct CellHash {
	std::size_t operator()(const Cell& c) const {
		return (static_cast<std::size_t>(static_cast<std::uint32_t>(c.x)) << 32)
			^ static_cast<std::uint32_t>(c.y);
	}
};

using HeightMap = std::unordered_map<Cell, float, CellHash>;

constexpr float kCellMm = 2.5f;
constexpr float kZMarginMm = 3.0f;

struct Stats {
	float min_z = 0.f;
	float max_z = 0.f;
	bool valid = false;
};

float PercentileZ(const pcl::PointCloud<pcl::PointXYZ>& cloud, float q) {
	std::vector<float> zs;
	zs.reserve(cloud.size());
	for (const auto& p : cloud.points) {
		if (std::isfinite(p.z)) {
			zs.push_back(p.z);
		}
	}
	if (zs.empty()) {
		return 0.f;
	}
	q = std::max(0.f, std::min(1.f, q));
	const std::size_t k = static_cast<std::size_t>(
		q * static_cast<float>(zs.size() - 1));
	auto it = zs.begin() + static_cast<std::ptrdiff_t>(k);
	std::nth_element(zs.begin(), it, zs.end());
	return *it;
}

HeightMap ProjectXy(const pcl::PointCloud<pcl::PointXYZ>& cloud) {
	HeightMap map;
	if (cloud.empty()) {
		return map;
	}
	const float inv = 1.f / kCellMm;
	map.reserve(cloud.size());
	for (const auto& p : cloud.points) {
		if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
			continue;
		}
		const Cell key{
			static_cast<int>(std::floor(p.x * inv)),
			static_cast<int>(std::floor(p.y * inv))};
		// From -Z toward +Z: smaller Z is closer, so height = -Z.
		const float height = -p.z;
		auto it = map.find(key);
		if (it == map.end()) {
			map.emplace(key, height);
		} else if (height > it->second) {
			it->second = height;
		}
	}
	return map;
}

Stats CloudStats(const pcl::PointCloud<pcl::PointXYZ>& cloud) {
	Stats s;
	s.min_z = PercentileZ(cloud, 0.10f);
	s.max_z = PercentileZ(cloud, 0.90f);
	if (s.max_z < s.min_z) {
		std::swap(s.max_z, s.min_z);
	}
	s.valid = cloud.size() > 0 && std::isfinite(s.min_z) && std::isfinite(s.max_z);
	return s;
}

// Pose-aligned cuboid (PCA OBB), not the world AABB. Filling a world AABB
// as a solid box pulls every XY cell to the closest corner, so a tilted
// part looks like the global top layer and hides true overlaps.
pcl::PointCloud<pcl::PointXYZ> FillObbVoxels(const pcl::PointCloud<pcl::PointXYZ>& cloud) {
	pcl::PointCloud<pcl::PointXYZ> filled;
	if (cloud.size() < 3) {
		return filled;
	}

	Eigen::Vector4f centroid;
	pcl::compute3DCentroid(cloud, centroid);
	Eigen::Matrix3f cov;
	pcl::computeCovarianceMatrixNormalized(cloud, centroid, cov);
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> ev(cov);
	if (ev.info() != Eigen::Success) {
		return filled;
	}
	Eigen::Matrix3f axes = ev.eigenvectors();
	if (axes.determinant() < 0.f) {
		axes.col(0) *= -1.f;
	}
	const Eigen::Vector3f c = centroid.head<3>();

	Eigen::Vector3f mn = Eigen::Vector3f::Constant(std::numeric_limits<float>::infinity());
	Eigen::Vector3f mx = Eigen::Vector3f::Constant(-std::numeric_limits<float>::infinity());
	for (const auto& p : cloud.points) {
		if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
			continue;
		}
		const Eigen::Vector3f local = axes.transpose() * (Eigen::Vector3f(p.x, p.y, p.z) - c);
		mn = mn.cwiseMin(local);
		mx = mx.cwiseMax(local);
	}
	if (!mn.allFinite() || !mx.allFinite()) {
		return filled;
	}
	for (int i = 0; i < 3; ++i) {
		if (mx[i] - mn[i] < kCellMm) {
			const float mid = 0.5f * (mx[i] + mn[i]);
			mn[i] = mid - 0.5f * kCellMm;
			mx[i] = mid + 0.5f * kCellMm;
		}
	}

	auto axis_n = [](float lo, float hi) {
		const int n = std::max(1, static_cast<int>(std::ceil((hi - lo) / kCellMm)));
		return std::min(n, 80);
	};
	const int nx = axis_n(mn.x(), mx.x());
	const int ny = axis_n(mn.y(), mx.y());
	const int nz = axis_n(mn.z(), mx.z());
	const Eigen::Vector3f step((mx.x() - mn.x()) / static_cast<float>(nx),
	                           (mx.y() - mn.y()) / static_cast<float>(ny),
	                           (mx.z() - mn.z()) / static_cast<float>(nz));
	filled.reserve(static_cast<std::size_t>(nx) * ny * nz);
	for (int ix = 0; ix < nx; ++ix) {
		for (int iy = 0; iy < ny; ++iy) {
			for (int iz = 0; iz < nz; ++iz) {
				const Eigen::Vector3f local(
					mn.x() + (ix + 0.5f) * step.x(),
					mn.y() + (iy + 0.5f) * step.y(),
					mn.z() + (iz + 0.5f) * step.z());
				const Eigen::Vector3f world = axes * local + c;
				pcl::PointXYZ p;
				p.x = world.x();
				p.y = world.y();
				p.z = world.z();
				filled.push_back(p);
			}
		}
	}
	return filled;
}

StackFilterResult Score(
	const std::vector<HeightMap>& maps,
	const std::vector<Stats>& stats,
	float overlap_threshold
) {
	const std::size_t n = maps.size();
	HeightMap global_max;
	for (std::size_t i = 0; i < n; ++i) {
		for (const auto& cell : maps[i]) {
			auto it = global_max.find(cell.first);
			if (it == global_max.end()) {
				global_max.emplace(cell.first, cell.second);
			} else if (cell.second > it->second) {
				it->second = cell.second;
			}
		}
	}

	StackFilterResult out;
	out.overlap_ratio.assign(n, 0.f);
	for (std::size_t i = 0; i < n; ++i) {
		if (maps[i].empty() || !stats[i].valid) {
			continue;
		}
		int stacked = 0;
		for (const auto& cell : maps[i]) {
			auto it = global_max.find(cell.first);
			if (it != global_max.end() && it->second > cell.second + kZMarginMm) {
				++stacked;
			}
		}
		out.overlap_ratio[i] =
			static_cast<float>(stacked) / static_cast<float>(maps[i].size());
		if (out.overlap_ratio[i] <= overlap_threshold) {
			out.kept.push_back(static_cast<int>(i));
		}
	}
	std::sort(out.kept.begin(), out.kept.end(), [&](int a, int b) {
		return stats[a].min_z < stats[b].min_z;
	});
	return out;
}

}  // namespace

StackFilterResult FilterStacked(
	const std::vector<pcl::PointCloud<pcl::PointXYZ>>& clouds,
	StackFilterMethod method,
	float overlap_threshold
) {
	const std::size_t n = clouds.size();
	std::vector<HeightMap> maps(n);
	std::vector<Stats> stats(n);
	for (std::size_t i = 0; i < n; ++i) {
		stats[i] = CloudStats(clouds[i]);
		if (method == StackFilterMethod::BoundingBox3D) {
			auto filled = FillObbVoxels(clouds[i]);
			maps[i] = ProjectXy(filled.empty() ? clouds[i] : filled);
		} else {
			maps[i] = ProjectXy(clouds[i]);
		}
	}
	return Score(maps, stats, overlap_threshold);
}

}  // namespace poser
