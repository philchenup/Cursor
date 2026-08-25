#include "stack_filter/stacked_object_filter.h"

#include <iostream>
#include <vector>

namespace {

pcl::PointCloud<pcl::PointXYZ> MakeBox(float cx, float cy, float cz, float size, float step) {
	pcl::PointCloud<pcl::PointXYZ> cloud;
	const float h = 0.5f * size;
	for (float x = cx - h; x <= cx + h; x += step) {
		for (float y = cy - h; y <= cy + h; y += step) {
			for (int iz = 0; iz < 2; ++iz) {
				pcl::PointXYZ p;
				p.x = x;
				p.y = y;
				p.z = cz + (iz == 0 ? h : -h);
				cloud.push_back(p);
			}
		}
	}
	return cloud;
}

}  // namespace

int main() {
	// Camera +Z down, millimetres. Smaller Z is closer to the camera.
	std::vector<pcl::PointCloud<pcl::PointXYZ>> clouds;
	clouds.push_back(MakeBox(0.f, 0.f, 400.f, 100.f, 4.f));    // 0 bottom, covered
	clouds.push_back(MakeBox(0.f, 0.f, 380.f, 100.f, 4.f));    // 1 top, same XY
	clouds.push_back(MakeBox(160.f, 0.f, 400.f, 100.f, 4.f));  // 2 side, free

	const auto result = poser::FilterStacked(clouds, 0.1f);

	std::cout << "overlap threshold = 0.1\n";
	for (std::size_t i = 0; i < clouds.size(); ++i) {
		std::cout << "cloud " << i
		          << "  overlap=" << result.overlap_ratio[i]
		          << "  " << (result.overlap_ratio[i] <= 0.1f ? "KEEP" : "DROP")
		          << "\n";
	}
	std::cout << "kept:";
	for (int idx : result.kept) {
		std::cout << " " << idx;
	}
	std::cout << "\n";
	return 0;
}
