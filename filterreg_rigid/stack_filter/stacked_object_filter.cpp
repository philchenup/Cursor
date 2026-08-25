#include "stack_filter/stacked_object_filter.h"

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
constexpr float kMinBandMm = 5.0f;

struct Stats {
	float min_z = 0.f;
	float max_z = 0.f;
	bool valid = false;
};

HeightMap ProjectFromOrigin(const pcl::PointCloud<pcl::PointXYZ>& cloud) {
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
		// Origin looks along +Z: closer to origin has smaller Z, larger height.
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
	s.min_z = std::numeric_limits<float>::infinity();
	s.max_z = -std::numeric_limits<float>::infinity();
	for (const auto& p : cloud.points) {
		if (!std::isfinite(p.z)) {
			continue;
		}
		s.min_z = std::min(s.min_z, p.z);
		s.max_z = std::max(s.max_z, p.z);
		s.valid = true;
	}
	return s;
}

float Median(std::vector<float> v) {
	if (v.empty()) {
		return kMinBandMm * 2.f;
	}
	auto mid = v.begin() + static_cast<std::ptrdiff_t>(v.size() / 2);
	std::nth_element(v.begin(), mid, v.end());
	return std::max(*mid, kMinBandMm);
}

pcl::PointCloud<pcl::PointXYZ> FillAabbVoxels(const pcl::PointCloud<pcl::PointXYZ>& cloud) {
	pcl::PointCloud<pcl::PointXYZ> filled;
	if (cloud.empty()) {
		return filled;
	}
	Eigen::Vector4f mn, mx;
	pcl::getMinMax3D(cloud, mn, mx);
	auto axis_n = [](float lo, float hi) {
		const int n = std::max(1, static_cast<int>(std::ceil((hi - lo) / kCellMm)));
		return std::min(n, 80);
	};
	const int nx = axis_n(mn.x(), mx.x());
	const int ny = axis_n(mn.y(), mx.y());
	const int nz = axis_n(mn.z(), mx.z());
	const float dx = (mx.x() - mn.x()) / static_cast<float>(nx);
	const float dy = (mx.y() - mn.y()) / static_cast<float>(ny);
	const float dz = (mx.z() - mn.z()) / static_cast<float>(nz);
	filled.reserve(static_cast<std::size_t>(nx) * ny * nz);
	for (int ix = 0; ix < nx; ++ix) {
		const float x = mn.x() + (ix + 0.5f) * dx;
		for (int iy = 0; iy < ny; ++iy) {
			const float y = mn.y() + (iy + 0.5f) * dy;
			for (int iz = 0; iz < nz; ++iz) {
				pcl::PointXYZ p;
				p.x = x;
				p.y = y;
				p.z = mn.z() + (iz + 0.5f) * dz;
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
	std::vector<float> thick;
	float z_top = std::numeric_limits<float>::infinity();
	for (std::size_t i = 0; i < n; ++i) {
		if (stats[i].valid) {
			z_top = std::min(z_top, stats[i].min_z);
			thick.push_back(std::max(stats[i].max_z - stats[i].min_z, kMinBandMm));
		}
		for (const auto& cell : maps[i]) {
			auto it = global_max.find(cell.first);
			if (it == global_max.end()) {
				global_max.emplace(cell.first, cell.second);
			} else if (cell.second > it->second) {
				it->second = cell.second;
			}
		}
	}
	const float layer_band = 0.5f * Median(thick);

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
		const bool top_layer = stats[i].min_z <= z_top + layer_band;
		const bool not_stacked = out.overlap_ratio[i] <= overlap_threshold;
		if (top_layer && not_stacked) {
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
			maps[i] = ProjectFromOrigin(FillAabbVoxels(clouds[i]));
		} else {
			maps[i] = ProjectFromOrigin(clouds[i]);
		}
	}
	return Score(maps, stats, overlap_threshold);
}

}  // namespace poser
