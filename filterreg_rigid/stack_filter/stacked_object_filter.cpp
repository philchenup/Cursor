#include "stack_filter/stacked_object_filter.h"

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

constexpr float kCellSizeMm = 2.5f;
constexpr float kHeightMarginMm = 3.0f;
constexpr float kMinLayerBandMm = 5.0f;

struct CloudStats {
	float min_z = 0.0f;
	float max_z = 0.0f;
	bool valid = false;
};

HeightMap ProjectXY(const pcl::PointCloud<pcl::PointXYZ>& cloud) {
	HeightMap map;
	if (cloud.empty()) {
		return map;
	}
	const float inv = 1.0f / kCellSizeMm;
	map.reserve(cloud.size());
	for (const auto& p : cloud.points) {
		if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
			continue;
		}
		const Cell key{
			static_cast<int>(std::floor(p.x * inv)),
			static_cast<int>(std::floor(p.y * inv))};
		const float height = -p.z;  // closer to camera = larger height
		auto it = map.find(key);
		if (it == map.end()) {
			map.emplace(key, height);
		} else if (height > it->second) {
			it->second = height;
		}
	}
	return map;
}

CloudStats ComputeStats(const pcl::PointCloud<pcl::PointXYZ>& cloud) {
	CloudStats s;
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

float MedianPositive(std::vector<float> values) {
	if (values.empty()) {
		return kMinLayerBandMm * 2.0f;
	}
	const auto mid = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2);
	std::nth_element(values.begin(), mid, values.end());
	return std::max(*mid, kMinLayerBandMm);
}

}  // namespace

StackFilterResult FilterStacked(
	const std::vector<pcl::PointCloud<pcl::PointXYZ>>& clouds,
	float overlap_threshold
) {
	const std::size_t n = clouds.size();
	std::vector<HeightMap> maps(n);
	std::vector<CloudStats> stats(n);
	HeightMap global_max;
	std::vector<float> thicknesses;
	thicknesses.reserve(n);

	float z_top = std::numeric_limits<float>::infinity();  // closest-to-camera min-z
	for (std::size_t i = 0; i < n; ++i) {
		maps[i] = ProjectXY(clouds[i]);
		stats[i] = ComputeStats(clouds[i]);
		if (stats[i].valid) {
			z_top = std::min(z_top, stats[i].min_z);
			thicknesses.push_back(std::max(stats[i].max_z - stats[i].min_z, kMinLayerBandMm));
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

	// Top layer = objects whose nearest point is within half a part-thickness
	// of the globally closest object. Anything deeper is a lower layer.
	const float layer_band = 0.5f * MedianPositive(thicknesses);

	StackFilterResult out;
	out.overlap_ratio.assign(n, 0.0f);

	for (std::size_t i = 0; i < n; ++i) {
		if (maps[i].empty() || !stats[i].valid) {
			continue;
		}
		int pressed = 0;
		for (const auto& cell : maps[i]) {
			auto it = global_max.find(cell.first);
			if (it != global_max.end() && it->second > cell.second + kHeightMarginMm) {
				++pressed;
			}
		}
		out.overlap_ratio[i] =
			static_cast<float>(pressed) / static_cast<float>(maps[i].size());

		const bool on_top_layer = stats[i].min_z <= z_top + layer_band;
		const bool uncovered = out.overlap_ratio[i] <= overlap_threshold;
		if (on_top_layer && uncovered) {
			out.kept.push_back(static_cast<int>(i));
		}
	}

	std::sort(out.kept.begin(), out.kept.end(), [&](int a, int b) {
		return stats[a].min_z < stats[b].min_z;
	});
	return out;
}

}  // namespace poser
