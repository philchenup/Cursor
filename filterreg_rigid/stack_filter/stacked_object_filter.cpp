#include "stack_filter/stacked_object_filter.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <utility>

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
		// Camera +Z down: height toward the camera is -Z.
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

}  // namespace

StackFilterResult FilterStacked(
	const std::vector<pcl::PointCloud<pcl::PointXYZ>>& clouds,
	float overlap_threshold
) {
	const std::size_t n = clouds.size();
	std::vector<HeightMap> maps(n);
	HeightMap global_max;
	for (std::size_t i = 0; i < n; ++i) {
		maps[i] = ProjectXY(clouds[i]);
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
	out.overlap_ratio.assign(n, 0.0f);
	std::vector<float> mean_height(n, 0.0f);

	for (std::size_t i = 0; i < n; ++i) {
		if (maps[i].empty()) {
			continue;
		}
		int pressed = 0;
		double height_sum = 0.0;
		for (const auto& cell : maps[i]) {
			height_sum += cell.second;
			auto it = global_max.find(cell.first);
			if (it != global_max.end() && it->second > cell.second + kHeightMarginMm) {
				++pressed;
			}
		}
		out.overlap_ratio[i] =
			static_cast<float>(pressed) / static_cast<float>(maps[i].size());
		mean_height[i] = static_cast<float>(height_sum / maps[i].size());
		if (out.overlap_ratio[i] <= overlap_threshold) {
			out.kept.push_back(static_cast<int>(i));
		}
	}

	std::sort(out.kept.begin(), out.kept.end(), [&](int a, int b) {
		return mean_height[a] > mean_height[b];
	});
	return out;
}

}  // namespace poser
