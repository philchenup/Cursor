#include "stack_filter/stacked_object_filter.h"

#include <pcl/visualization/pcl_visualizer.h>

#include <iostream>
#include <string>
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

pcl::PointCloud<pcl::PointXYZRGB>::Ptr ColorCloud(
	const pcl::PointCloud<pcl::PointXYZ>& src,
	std::uint8_t r,
	std::uint8_t g,
	std::uint8_t b
) {
	auto out = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	out->reserve(src.size());
	for (const auto& p : src.points) {
		pcl::PointXYZRGB q;
		q.x = p.x;
		q.y = p.y;
		q.z = p.z;
		q.r = r;
		q.g = g;
		q.b = b;
		out->push_back(q);
	}
	return out;
}

}  // namespace

int main() {
	// Camera +Z down, millimetres. Smaller Z is closer to the camera.
	std::vector<pcl::PointCloud<pcl::PointXYZ>> clouds;
	clouds.push_back(MakeBox(0.f, 0.f, 400.f, 100.f, 4.f));    // 0 bottom, covered
	clouds.push_back(MakeBox(0.f, 0.f, 380.f, 100.f, 4.f));    // 1 top, same XY
	clouds.push_back(MakeBox(160.f, 0.f, 400.f, 100.f, 4.f));  // 2 side, free

	const auto result = poser::FilterStacked(clouds, 0.1f);

	std::vector<char> keep(clouds.size(), 0);
	for (int idx : result.kept) {
		keep[static_cast<std::size_t>(idx)] = 1;
	}

	std::cout << "overlap threshold = 0.1\n";
	for (std::size_t i = 0; i < clouds.size(); ++i) {
		std::cout << "cloud " << i
		          << "  overlap=" << result.overlap_ratio[i]
		          << "  " << (keep[i] ? "KEEP" : "DROP")
		          << "\n";
	}
	std::cout << "kept:";
	for (int idx : result.kept) {
		std::cout << " " << idx;
	}
	std::cout << "\n";

	pcl::visualization::PCLVisualizer viewer("stack filter  keep=green  drop=red");
	viewer.setBackgroundColor(0.08, 0.08, 0.10);
	viewer.addCoordinateSystem(50.0);
	viewer.initCameraParameters();

	for (std::size_t i = 0; i < clouds.size(); ++i) {
		const bool kept = keep[i] != 0;
		auto rgb = ColorCloud(
			clouds[i],
			kept ? 32 : 210,
			kept ? 180 : 50,
			kept ? 80 : 50);
		const std::string id = "cloud_" + std::to_string(i);
		viewer.addPointCloud<pcl::PointXYZRGB>(rgb, id);
		viewer.setPointCloudRenderingProperties(
			pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, id);
	}

	// Oblique view so the covered (red) plate under the green one is visible.
	viewer.setCameraPosition(-220.0, -520.0, 80.0, 80.0, 0.0, 390.0, 0.0, 0.0, -1.0);
	viewer.spinOnce(100);
	viewer.saveScreenshot("stack_filter_vis.png");
	while (!viewer.wasStopped()) {
		viewer.spinOnce(16);
	}
	return 0;
}
