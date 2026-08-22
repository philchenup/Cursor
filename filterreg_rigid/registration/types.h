#pragma once

#include <Eigen/Dense>
#include <cstddef>
#include <vector>

namespace small_reg {

enum class Method { ICP, PlaneICP, GICP };

struct PointCloud {
	std::vector<Eigen::Vector3d> points;
	std::vector<Eigen::Vector3d> normals;
	std::vector<Eigen::Matrix3d> covs;

	std::size_t size() const { return points.size(); }
	bool empty() const { return points.empty(); }
	bool has_normals() const { return normals.size() == points.size(); }
	bool has_covs() const { return covs.size() == points.size(); }
};

struct AlignSetting {
	Method method = Method::GICP;
	double voxel = 0.05;
	double max_corr_dist = 0.5;
	double min_corr_dist = 0.0;  // 0 => 3 * voxel
	int max_iterations = 40;
	int cov_knn = 20;
	// Smallest covariance eigenvalue is clamped to cov_clip_ratio * λ_max.
	// Smaller values make GICP more accurate on clean planes but far more
	// sensitive to a slightly too-large max_corr_dist.
	double cov_clip_ratio = 1e-3;
	double rotation_eps = 0.1 * 3.14159265358979323846 / 180.0;
	double translation_eps = 1e-4;
	double lm_lambda_init = 1e-3;
	bool adaptive_max_dist = true;
	double adaptive_scale = 2.5;
	bool robust_kernel = true;
	double robust_k = 0.0;  // 0 => auto (GICP: 1.0 whitened, ICP/P2L: 2*voxel m)
	bool normal_check = true;
	double min_normal_cos = 0.5;  // reject if n_t · R n_s < this (60 deg)
	bool multiscale = true;
	double min_inlier_ratio = 0.05;
	bool verbose = false;
};

struct AlignResult {
	Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
	int iterations = 0;
	int num_inliers = 0;
	int num_source = 0;
	double fitness = 0.0;       // mean inlier residual (meters or Mahalanobis)
	double overlap = 0.0;       // inliers / source size
	double final_max_dist = 0.0;
	bool converged = false;
	bool success = false;
};

inline const char* MethodName(Method m) {
	switch (m) {
	case Method::ICP:
		return "ICP(P2P)";
	case Method::PlaneICP:
		return "ICP(P2L)";
	case Method::GICP:
		return "GICP";
	}
	return "unknown";
}

}  // namespace small_reg
