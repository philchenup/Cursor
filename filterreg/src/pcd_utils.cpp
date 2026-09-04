#include "pcd_utils.h"

#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>
#include <small_gicp/pcl/pcl_point.hpp>
#include <small_gicp/pcl/pcl_point_traits.hpp>
#include <small_gicp/util/downsampling_omp.hpp>

#include <cmath>

namespace PcdUtils {

	void pcd_voxel_down(const ct::Cloud::Ptr& cloud_in, ct::Cloud::Ptr& cloud_out, double radius) {
		pcl::VoxelGrid<ct::PointXYZRGBN> vfilter;
		vfilter.setInputCloud(cloud_in);
		vfilter.setLeafSize(radius, radius, radius);
		vfilter.setFilterLimitsNegative(false);
		vfilter.filter(*cloud_out);
	}

	void pcd_voxel_down_omp(const ct::Cloud::Ptr& cloud_in, ct::Cloud::Ptr& cloud_out, double radius) {
		const auto& in_pcl = static_cast<const pcl::PointCloud<ct::PointXYZRGBN>&>(*cloud_in);
		auto downsampled = small_gicp::voxelgrid_sampling_omp(in_pcl, radius, 8);
		cloud_out.reset(new ct::Cloud);
		pcl::copyPointCloud(*downsampled, *cloud_out);
	}

	void pcd_pass_filters(const ct::Cloud::Ptr& cloud_in, ct::Cloud::Ptr& cloud_out, double min_v, double max_v) {
		pcl::PassThrough<ct::PointXYZRGBN> pfilter;
		pfilter.setInputCloud(cloud_in);
		pfilter.setFilterFieldName("z");
		pfilter.setFilterLimits(min_v, max_v);
		pfilter.setNegative(false);
		pfilter.filter(*cloud_out);
	}

	double pcd_calculateFitness(const ct::Cloud::Ptr& source, const ct::Cloud::Ptr& target,
		double max_correspondence_distance) {
		if (source->empty() || target->empty()) {
			return 100.0;
		}
		pcl::KdTreeFLANN<pcl::PointXYZRGBNormal> kdtree;
		kdtree.setInputCloud(target);

		int inlier_count = 0;
		for (const auto& point : source->points) {
			std::vector<int> pointIdxNKNSearch(1);
			std::vector<float> pointNKNSquaredDistance(1);
			if (kdtree.nearestKSearch(point, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
				if (pointNKNSquaredDistance[0] < max_correspondence_distance * max_correspondence_distance) {
					inlier_count++;
				}
			}
		}
		double fitness = static_cast<double>(inlier_count) / static_cast<double>(source->size());

		return fitness;
	}

	void recoverPcdFromDepth(const cv::Mat Depth, const cv::Mat Color, const cv::Vec4f intrinsicArray, ct::Cloud::Ptr& pcd) {
		if (Depth.empty() || Color.empty()) return;
		float fx = intrinsicArray[2];
		float fy = intrinsicArray[3];
		float cx = intrinsicArray[0];
		float cy = intrinsicArray[1];
		for (int i = 0; i < Depth.rows; ++i) {          // ✅ i = row
			for (int j = 0; j < Depth.cols; ++j) {      // ✅ j = col
				float z = Depth.at<float>(i, j);
				if (!std::isfinite(z) || z <= 10.0f) {
					continue;
				}

				pcl::PointXYZRGBNormal p;
				p.x = (j - cx) * z / fx;
				p.y = (i - cy) * z / fy;
				p.z = z;

				const cv::Vec3b& bgr = Color.at<cv::Vec3b>(i, j); 
				p.r = bgr[2];
				p.g = bgr[1];
				p.b = bgr[0];

				pcd->push_back(p);
			}
		}
		pcd->width = pcd->size();
		pcd->height = 1;
		pcd->is_dense = true;
	}

	void pcd_radius_filter(const ct::Cloud::Ptr& cloud_in, ct::Cloud::Ptr& cloud_out, double radius, int min_pts)
	{
		if (cloud_in->empty()) return;

		pcl::KdTreeFLANN<pcl::PointXYZRGBNormal> kdtree;
		kdtree.setInputCloud(cloud_in);
		const float r2 = radius * radius;
		const int   K = min_pts + 1;      // +1：最近邻里包含自身

		const int N = static_cast<int>(cloud_in->size());
		std::vector<char> keep(N, 0);

#pragma omp parallel
		{
			std::vector<int>   idx(K);
			std::vector<float> d2(K);
#pragma omp for schedule(static)
			for (int i = 0; i < N; ++i) {
				const auto& p = cloud_in->points[i];
				if (std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z)) continue;
				if (kdtree.nearestKSearch(p, K, idx, d2) == K) {
					if (d2[K - 1] <= r2) keep[i] = 1;   // 半径内 ≥ minNeighbors -> 保留
				}
			}
		}

		cloud_out->reserve(N);
		for (int i = 0; i < N; ++i)
			if (keep[i]) cloud_out->push_back(cloud_in->points[i]);
		cloud_out->width = static_cast<uint32_t>(cloud_out->size());
		cloud_out->height = 1;
		cloud_out->is_dense = false;
	}

	void FilterReg(const ct::Cloud::Ptr& cloud_src, const ct::Cloud::Ptr& cloud_in, const regPara& rp, Eigen::Affine3f& T_res) {

		filterreg::Options opt;
		opt.objective = filterreg::Objective::PointToPoint;
		opt.update_sigma2 = true;
		opt.max_iter = rp.initRegIter;

		double kMmPerM = 1000.0;
		Eigen::MatrixX3f source_mm(cloud_src->size(), 3);
		for (size_t i = 0; i < cloud_src->size(); ++i) {
			source_mm(i, 0) = cloud_src->points[i].x;
			source_mm(i, 1) = cloud_src->points[i].y;
			source_mm(i, 2) = cloud_src->points[i].z;
		}

		Eigen::MatrixX3f target_mm(cloud_in->size(), 3);
		for (size_t i = 0; i < cloud_in->size(); ++i) {
			target_mm(i, 0) = cloud_in->points[i].x;
			target_mm(i, 1) = cloud_in->points[i].y;
			target_mm(i, 2) = cloud_in->points[i].z;
		}

		filterreg::Result res = filterreg::registration(
			source_mm / kMmPerM, target_mm / kMmPerM, {}, opt);
		res.transformation.t *= kMmPerM;
		
		Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
		T.block<3, 3>(0, 0) = res.transformation.rot;   // 左上角 3x3 = 旋转矩阵
		T.block<3, 1>(0, 3) = res.transformation.t;     // 右上角 3x1 = 平移向量

		ct::Cloud::Ptr initRegPcd(new ct::Cloud);
		pcl::transformPointCloud(*cloud_src, *initRegPcd, T);

		ct::Cloud::Ptr ail_cloud(new ct::Cloud);
		pcl::IterativeClosestPoint<ct::PointXYZRGBN, ct::PointXYZRGBN> icp;
		icp.setInputSource(initRegPcd);
		icp.setInputTarget(cloud_in);
		icp.setMaxCorrespondenceDistance(rp.maxCorrDis);   
		icp.setMaximumIterations(rp.refineIterations);
		icp.setTransformationEpsilon(1e-6);
		icp.setEuclideanFitnessEpsilon(1e-4);

		icp.align(*ail_cloud);

		if (icp.hasConverged()) {
			T_res = icp.getFinalTransformation() * T;
		}
		else {
			T_res = Eigen::Affine3f::Identity();
		}
	}

	std::vector<Eigen::Vector4f> pclToEigen4f(const ct::Cloud::Ptr& cloud)
	{
		std::vector<Eigen::Vector4f> points;
		points.reserve(cloud->size());
		for (const auto& p : cloud->points) {
			if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
				continue;
			points.emplace_back(p.x, p.y, p.z, 1.0f);
		}
		return points;
	}

	void removeBackground(const ct::Cloud::Ptr& background, const ct::Cloud::Ptr& scene, ct::Cloud::Ptr& filter, float resolution)
	{
		pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZRGBNormal> octree(resolution);

		octree.setInputCloud(background);
		octree.addPointsFromInputCloud();
		octree.switchBuffers();       

		octree.setInputCloud(scene);
		octree.addPointsFromInputCloud();

		std::vector<int> newIdx;
		octree.getPointIndicesFromNewVoxels(newIdx);

		pcl::copyPointCloud(*scene, newIdx, *filter);
		filter->is_dense = false;
	}
};