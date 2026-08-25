#include "stack_filter/stacked_object_filter.h"

#include <pcl/visualization/pcl_visualizer.h>

#include <iostream>
#include <string>
#include <vector>

namespace {

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
	const pcl::PointCloud<pcl::PointXYZ>& src, bool kept
) {
	auto out = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	out->reserve(src.size());
	const std::uint8_t r = kept ? 32 : 210;
	const std::uint8_t g = kept ? 180 : 50;
	const std::uint8_t b = kept ? 80 : 50;
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

void PrintResult(const char* name, const poser::StackFilterResult& result, std::size_t n) {
	std::vector<char> keep(n, 0);
	for (int idx : result.kept) {
		keep[static_cast<std::size_t>(idx)] = 1;
	}
	std::cout << name << "\n";
	for (std::size_t i = 0; i < n; ++i) {
		std::cout << "  cloud " << i
		          << "  overlap=" << result.overlap_ratio[i]
		          << "  " << (keep[i] ? "KEEP" : "DROP") << "\n";
	}
	std::cout << "  kept:";
	for (int idx : result.kept) {
		std::cout << " " << idx;
	}
	std::cout << "\n";
}

void AddClouds(
	pcl::visualization::PCLVisualizer& viewer,
	const std::vector<pcl::PointCloud<pcl::PointXYZ>>& clouds,
	const poser::StackFilterResult& result,
	int viewport,
	const std::string& prefix
) {
	std::vector<char> keep(clouds.size(), 0);
	for (int idx : result.kept) {
		keep[static_cast<std::size_t>(idx)] = 1;
	}
	for (std::size_t i = 0; i < clouds.size(); ++i) {
		const std::string id = prefix + std::to_string(i);
		viewer.addPointCloud<pcl::PointXYZRGB>(ColorCloud(clouds[i], keep[i] != 0), id, viewport);
		viewer.setPointCloudRenderingProperties(
			pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, id, viewport);
	}
}

}  // namespace

int main() {
	// Origin looks along +Z. Smaller Z is the top layer.
	// 0 lower, stacked under 1 -> DROP
	// 1 top layer              -> KEEP
	// 2 top layer, free        -> KEEP
	// 3 lower layer, free      -> DROP (cannot grasp)
	std::vector<pcl::PointCloud<pcl::PointXYZ>> clouds;
	clouds.push_back(MakePlate(0.f, 0.f, 340.f, 80.f, 8.f, 3.f));
	clouds.push_back(MakePlate(0.f, 0.f, 328.f, 80.f, 8.f, 3.f));
	clouds.push_back(MakePlate(120.f, 0.f, 329.f, 80.f, 8.f, 3.f));
	clouds.push_back(MakePlate(240.f, 0.f, 350.f, 80.f, 8.f, 3.f));

	const auto r2d = poser::FilterStacked(
		clouds, poser::StackFilterMethod::Projection2D, 0.1f);
	const auto r3d = poser::FilterStacked(
		clouds, poser::StackFilterMethod::BoundingBox3D, 0.1f);
	PrintResult("Projection2D", r2d, clouds.size());
	PrintResult("BoundingBox3D", r3d, clouds.size());

	pcl::visualization::PCLVisualizer viewer("origin +Z  green=KEEP  red=DROP");
	int vp_2d = 0;
	int vp_3d = 0;
	viewer.createViewPort(0.0, 0.0, 0.5, 1.0, vp_2d);
	viewer.createViewPort(0.5, 0.0, 1.0, 1.0, vp_3d);
	viewer.setBackgroundColor(0.08, 0.08, 0.10, vp_2d);
	viewer.setBackgroundColor(0.08, 0.08, 0.10, vp_3d);
	viewer.addText("Projection 2D", 10, 10, 16, 1, 1, 1, "txt2d", vp_2d);
	viewer.addText("BoundingBox 3D", 10, 10, 16, 1, 1, 1, "txt3d", vp_3d);
	viewer.addCoordinateSystem(40.0, "axes2d", vp_2d);
	viewer.addCoordinateSystem(40.0, "axes3d", vp_3d);

	AddClouds(viewer, clouds, r2d, vp_2d, "p2d_");
	AddClouds(viewer, clouds, r3d, vp_3d, "b3d_");

	viewer.initCameraParameters();
	viewer.setCameraPosition(-180.0, -480.0, 120.0, 120.0, 0.0, 335.0, 0.0, 0.0, -1.0);
	viewer.spinOnce(100);
	viewer.saveScreenshot("stack_filter_vis.png");
	while (!viewer.wasStopped()) {
		viewer.spinOnce(16);
	}
	return 0;
}
