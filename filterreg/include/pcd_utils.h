#pragma once

#ifndef PCD_UTILS_H
#define PCD_UTILS_H

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <pcl/registration/icp.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "cloud.h"
#include "filterreg.h"

struct regPara
{
	double initRegIter = 0.0;
	double init_sigma = 0.0;
	double maxCorrDis = 0.0;
	double refineIterations = 0.0;
};

namespace PcdUtils {

	enum Method { FICP, RICP ,PPL, RPPL};

	void pcd_voxel_down(const ct::Cloud::Ptr& cloud_in, ct::Cloud::Ptr& cloud_out, double radius);

	void pcd_voxel_down_omp(const ct::Cloud::Ptr& cloud_in, ct::Cloud::Ptr& cloud_out, double radius);

	void pcd_pass_filters(const ct::Cloud::Ptr& cloud_in, ct::Cloud::Ptr& cloud_out, double min_v, double max_v);

	double pcd_calculateFitness(const ct::Cloud::Ptr& source, const ct::Cloud::Ptr& target,
		double max_correspondence_distance);

	void pcd_radius_filter(const ct::Cloud::Ptr& cloud_in, ct::Cloud::Ptr& cloud_out, double radius, int min_pts);

	void FilterReg(const ct::Cloud::Ptr& cloud_src, const ct::Cloud::Ptr& cloud_in, const regPara& rp, Eigen::Affine3f& T_res);

	std::vector<Eigen::Vector4f> pclToEigen4f(const ct::Cloud::Ptr& cloud);

	void removeBackground(const ct::Cloud::Ptr& background, const ct::Cloud::Ptr& scene, ct::Cloud::Ptr& filter, float resolution = 2.0f);
};


#endif