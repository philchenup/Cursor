#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>

namespace poser {
namespace demo {

inline pcl::PointCloud<pcl::PointXYZ>::Ptr MakeBoxSurfaceCloud(
	float size_x,
	float size_y,
	float size_z,
	float spacing
) {
	auto cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(
		new pcl::PointCloud<pcl::PointXYZ>());
	spacing = std::max(spacing, 1e-4f);
	const float hx = 0.5f * size_x;
	const float hy = 0.5f * size_y;
	const float hz = 0.5f * size_z;

	auto add_face = [&](
		const Eigen::Vector3f& origin,
		const Eigen::Vector3f& du,
		const Eigen::Vector3f& dv,
		float len_u,
		float len_v
	) {
		const int nu = std::max(2, static_cast<int>(std::ceil(len_u / spacing)) + 1);
		const int nv = std::max(2, static_cast<int>(std::ceil(len_v / spacing)) + 1);
		for (int i = 0; i < nu; ++i) {
			const float u = (nu == 1) ? 0.0f : static_cast<float>(i) / static_cast<float>(nu - 1);
			for (int j = 0; j < nv; ++j) {
				const float v = (nv == 1) ? 0.0f : static_cast<float>(j) / static_cast<float>(nv - 1);
				const Eigen::Vector3f p = origin + u * du + v * dv;
				pcl::PointXYZ pt;
				pt.x = p.x();
				pt.y = p.y();
				pt.z = p.z();
				cloud->points.push_back(pt);
			}
		}
	};

	// Six faces of an axis-aligned box centered at the origin.
	add_face(Eigen::Vector3f(-hx, -hy, hz), Eigen::Vector3f(size_x, 0, 0), Eigen::Vector3f(0, size_y, 0), size_x, size_y);
	add_face(Eigen::Vector3f(-hx, -hy, -hz), Eigen::Vector3f(size_x, 0, 0), Eigen::Vector3f(0, size_y, 0), size_x, size_y);
	add_face(Eigen::Vector3f(-hx, hy, -hz), Eigen::Vector3f(size_x, 0, 0), Eigen::Vector3f(0, 0, size_z), size_x, size_z);
	add_face(Eigen::Vector3f(-hx, -hy, -hz), Eigen::Vector3f(size_x, 0, 0), Eigen::Vector3f(0, 0, size_z), size_x, size_z);
	add_face(Eigen::Vector3f(hx, -hy, -hz), Eigen::Vector3f(0, size_y, 0), Eigen::Vector3f(0, 0, size_z), size_y, size_z);
	add_face(Eigen::Vector3f(-hx, -hy, -hz), Eigen::Vector3f(0, size_y, 0), Eigen::Vector3f(0, 0, size_z), size_y, size_z);

	cloud->width = static_cast<std::uint32_t>(cloud->points.size());
	cloud->height = 1;
	cloud->is_dense = true;
	return cloud;
}

inline Eigen::Matrix4f MakePose(float x, float y, float z, float yaw_rad = 0.0f) {
	Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
	if (std::abs(yaw_rad) > 1e-8f) {
		pose.block<3, 3>(0, 0) =
			Eigen::AngleAxisf(yaw_rad, Eigen::Vector3f::UnitZ()).toRotationMatrix();
	}
	pose(0, 3) = x;
	pose(1, 3) = y;
	pose(2, 3) = z;
	return pose;
}

// Camera-down flange template in millimetres: annulus, 4 bolt holes, rim notch.
inline pcl::PointCloud<pcl::PointXYZ>::Ptr MakeFlangeCloud(
	float outer_r = 55.0f,
	float inner_r = 18.0f,
	float thickness = 12.0f,
	float spacing = 2.0f,
	float bolt_r = 5.0f,
	float bolt_circle_r = 38.0f,
	int n_bolts = 4,
	float notch_r = 12.0f
) {
	auto cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(
		new pcl::PointCloud<pcl::PointXYZ>());
	spacing = std::max(spacing, 0.5f);
	const float hz = 0.5f * thickness;
	const float kPi = 3.14159265f;

	auto in_solid = [&](float x, float y) {
		const float r2 = x * x + y * y;
		if (r2 > outer_r * outer_r || r2 < inner_r * inner_r) {
			return false;
		}
		for (int k = 0; k < n_bolts; ++k) {
			const float a = static_cast<float>(k) * 2.0f * kPi / static_cast<float>(n_bolts);
			const float bx = bolt_circle_r * std::cos(a);
			const float by = bolt_circle_r * std::sin(a);
			const float dx = x - bx;
			const float dy = y - by;
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

	const int n = std::max(8, static_cast<int>(std::ceil(2.0f * outer_r / spacing)));
	for (int iz = 0; iz < 2; ++iz) {
		const float z = (iz == 0) ? hz : -hz;
		for (int ix = 0; ix <= n; ++ix) {
			const float x = -outer_r + static_cast<float>(ix) * spacing;
			for (int iy = 0; iy <= n; ++iy) {
				const float y = -outer_r + static_cast<float>(iy) * spacing;
				if (!in_solid(x, y)) {
					continue;
				}
				pcl::PointXYZ pt;
				pt.x = x;
				pt.y = y;
				pt.z = z;
				cloud->points.push_back(pt);
			}
		}
	}

	cloud->width = static_cast<std::uint32_t>(cloud->points.size());
	cloud->height = 1;
	cloud->is_dense = true;
	return cloud;
}

inline std::string KeepLabel(bool kept) {
	return kept ? "KEEP" : "DROP";
}

}  // namespace demo
}  // namespace poser
