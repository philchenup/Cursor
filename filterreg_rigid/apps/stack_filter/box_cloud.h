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

inline Eigen::Matrix4f MakePose(float x, float y, float z) {
	Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
	pose(0, 3) = x;
	pose(1, 3) = y;
	pose(2, 3) = z;
	return pose;
}

inline std::string KeepLabel(bool kept) {
	return kept ? "KEEP" : "DROP";
}

}  // namespace demo
}  // namespace poser
