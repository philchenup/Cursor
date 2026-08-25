#include "stack_filter/stacked_object_filter.h"

#include <pcl/visualization/pcl_visualizer.h>

#include <iostream>
#include <string>
#include <vector>

namespace {

// Thin plate in millimetres. cz is the plate centre along camera Z.
pcl::PointCloud<pcl::PointXYZ> MakePlate(
	float cx, float cy, float cz, float size_xy, float thickness, float step
) {
	pcl::PointCloud<pcl::PointXYZ> cloud;
	const float hxy = 0.5f * size_xy;
	const float hz = 0.5f * thickness;
	for (float x = cx - hxy; x <= cx + hxy; x += step) {
		for (float y = cy - hxy; y <= cy + hxy; y += step) {
			for (int iz = 0; iz < 2; ++iz) {
				pcl::PointXYZ p;
				p.x = x;
				p.y = y;
				p.z = cz + (iz == 0 ? hz : -hz);
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
	// Camera +Z down, mm. Smaller Z = closer = top layer.
	// 0: lower plate under 1  -> DROP (lower layer, pressed)
	// 1: top plate            -> KEEP
	// 2: same height as 1     -> KEEP (top layer, not pressed)
	// 3: table-level, free    -> DROP (lower layer, cannot grasp)
	std::vector<pcl::PointCloud<pcl::PointXYZ>> clouds;
	clouds.push_back(MakePlate(0.f, 0.f, 340.f, 80.f, 8.f, 3.f));
	clouds.push_back(MakePlate(0.f, 0.f, 328.f, 80.f, 8.f, 3.f));
	clouds.push_back(MakePlate(120.f, 0.f, 329.f, 80.f, 8.f, 3.f));
	clouds.push_back(MakePlate(240.f, 0.f, 350.f, 80.f, 8.f, 3.f));

	const auto result = poser::FilterStacked(clouds, 0.1f);

	std::vector<char> keep(clouds.size(), 0);
	for (int idx : result.kept) {
		keep[static_cast<std::size_t>(idx)] = 1;
	}

	std::cout << "top-layer + uncovered, threshold=0.1\n";
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

	pcl::visualization::PCLVisualizer viewer("top layer  keep=green  drop=red");
	viewer.setBackgroundColor(0.08, 0.08, 0.10);
	viewer.addCoordinateSystem(40.0);
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

	viewer.setCameraPosition(-180.0, -480.0, 120.0, 120.0, 0.0, 335.0, 0.0, 0.0, -1.0);
	viewer.spinOnce(100);
	viewer.saveScreenshot("stack_filter_vis.png");
	while (!viewer.wasStopped()) {
		viewer.spinOnce(16);
	}
	return 0;
}
