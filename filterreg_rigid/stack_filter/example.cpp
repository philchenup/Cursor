#include "stack_filter/stacked_object_filter.h"

#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

namespace {

pcl::PointCloud<pcl::PointXYZ> MakeFlange() {
	pcl::PointCloud<pcl::PointXYZ> cloud;
	const float outer_r = 55.f;
	const float inner_r = 18.f;
	const float hz = 6.f;
	const float spacing = 2.f;
	const float bolt_r = 5.f;
	const float bolt_circle_r = 38.f;
	const float notch_r = 12.f;
	const float kPi = 3.14159265f;
	auto in_solid = [&](float x, float y) {
		const float r2 = x * x + y * y;
		if (r2 > outer_r * outer_r || r2 < inner_r * inner_r) {
			return false;
		}
		for (int k = 0; k < 4; ++k) {
			const float a = static_cast<float>(k) * 0.5f * kPi;
			const float dx = x - bolt_circle_r * std::cos(a);
			const float dy = y - bolt_circle_r * std::sin(a);
			if (dx * dx + dy * dy < bolt_r * bolt_r) {
				return false;
			}
		}
		const float nx = outer_r - 0.35f * notch_r;
		const float dxn = x - nx;
		if (dxn * dxn + y * y < notch_r * notch_r && x > nx - notch_r) {
			return false;
		}
		return true;
	};
	const int n = static_cast<int>(std::ceil(2.f * outer_r / spacing));
	for (int iz = 0; iz < 2; ++iz) {
		const float z = (iz == 0) ? hz : -hz;
		for (int ix = 0; ix <= n; ++ix) {
			const float x = -outer_r + static_cast<float>(ix) * spacing;
			for (int iy = 0; iy <= n; ++iy) {
				const float y = -outer_r + static_cast<float>(iy) * spacing;
				if (!in_solid(x, y)) {
					continue;
				}
				pcl::PointXYZ p;
				p.x = x;
				p.y = y;
				p.z = z;
				cloud.push_back(p);
			}
		}
	}
	return cloud;
}

pcl::PointCloud<pcl::PointXYZ> Place(
	const pcl::PointCloud<pcl::PointXYZ>& model,
	float x,
	float y,
	float z,
	float yaw,
	float pitch = 0.f,
	float roll = 0.f
) {
	Eigen::Affine3f t = Eigen::Affine3f::Identity();
	t.translate(Eigen::Vector3f(x, y, z));
	t.rotate(Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()));
	t.rotate(Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()));
	t.rotate(Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()));
	pcl::PointCloud<pcl::PointXYZ> out;
	pcl::transformPointCloud(model, out, t);
	return out;
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
	std::vector<std::size_t> order(clouds.size());
	for (std::size_t i = 0; i < order.size(); ++i) {
		order[i] = i;
	}
	// Farther (larger Z) first so closer clouds paint on top.
	std::sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) {
		float za = 0.f, zb = 0.f;
		for (const auto& p : clouds[a].points) {
			za += p.z;
		}
		for (const auto& p : clouds[b].points) {
			zb += p.z;
		}
		za /= static_cast<float>(std::max<std::size_t>(clouds[a].size(), 1));
		zb /= static_cast<float>(std::max<std::size_t>(clouds[b].size(), 1));
		return za > zb;
	});
	for (std::size_t i : order) {
		const std::string id = prefix + std::to_string(i);
		viewer.addPointCloud<pcl::PointXYZRGB>(ColorCloud(clouds[i], keep[i] != 0), id, viewport);
		viewer.setPointCloudRenderingProperties(
			pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, id, viewport);
		pcl::PointXYZ centroid(0.f, 0.f, 0.f);
		for (const auto& p : clouds[i].points) {
			centroid.x += p.x;
			centroid.y += p.y;
			centroid.z += p.z;
		}
		const float n = static_cast<float>(std::max<std::size_t>(clouds[i].size(), 1));
		centroid.x /= n;
		centroid.y /= n;
		centroid.z /= n;
		viewer.addText3D(
			std::to_string(i), centroid, 12.0, 1.0, 1.0, 1.0,
			prefix + "lb" + std::to_string(i), viewport);
	}
}

}  // namespace

int main() {
	// Five φ110 mm flanges. View from -Z toward +Z; smaller Z is closer.
	// Drop only when a closer object covers the XY footprint.
	// 0 top-left   isolated, closer     -> KEEP
	// 1 top-right  isolated, farther    -> KEEP (not covered)
	// 2 bot-left   under 3              -> DROP
	// 3 center     on top of 2 and 4    -> KEEP
	// 4 bot-right  under 3              -> DROP
	const auto model = MakeFlange();
	std::vector<pcl::PointCloud<pcl::PointXYZ>> clouds;
	clouds.push_back(Place(model, -95.f, 85.f, 322.f, -0.20f));
	clouds.push_back(Place(model, 95.f, 85.f, 348.f, 0.15f));
	clouds.push_back(Place(model, -55.f, -75.f, 340.f, 0.40f));
	clouds.push_back(Place(model, 5.f, -35.f, 324.f, 0.10f));
	clouds.push_back(Place(model, 70.f, -75.f, 342.f, -0.25f));

	const auto r2d = poser::FilterStacked(
		clouds, poser::StackFilterMethod::Projection2D, 0.1f);
	const auto r3d = poser::FilterStacked(
		clouds, poser::StackFilterMethod::BoundingBox3D, 0.1f);
	PrintResult("Projection2D", r2d, clouds.size());
	PrintResult("BoundingBox3D", r3d, clouds.size());

	pcl::visualization::PCLVisualizer viewer("-Z -> +Z  green=KEEP  red=DROP");
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
	// XY top-down so +X is right and +Y is up (cloud 1 at upper-right).
	// Filter depth is still -Z toward +Z: smaller Z is closer.
	viewer.setCameraPosition(0.0, 0.0, 700.0, 0.0, 0.0, 330.0, 0.0, 1.0, 0.0);
	viewer.spinOnce(100);
	viewer.saveScreenshot("stack_filter_vis.png");
	while (!viewer.wasStopped()) {
		viewer.spinOnce(16);
	}
	return 0;
}
