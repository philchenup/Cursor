#include "registration/align.h"

#include "registration/kdtree.h"
#include "registration/preprocess.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

namespace small_reg {
namespace {

inline Eigen::Matrix3d Skew(const Eigen::Vector3d& v) {
	Eigen::Matrix3d m;
	m << 0.0, -v.z(), v.y(),
	     v.z(), 0.0, -v.x(),
	     -v.y(), v.x(), 0.0;
	return m;
}

// Left-multiply se(3) exponential, matching small_gicp's LM update T ← exp(x) T.
Eigen::Isometry3d Se3Exp(const Eigen::Matrix<double, 6, 1>& x) {
	const Eigen::Vector3d omega = x.head<3>();
	const Eigen::Vector3d tau = x.tail<3>();
	const double theta = omega.norm();
	Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
	if (theta < 1e-12) {
		R += Skew(omega);
	} else {
		R = Eigen::AngleAxisd(theta, omega / theta).toRotationMatrix();
	}
	Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
	T.linear() = R;
	T.translation() = tau;
	return T;
}

inline double CauchyWeight(double residual_abs, double k) {
	if (k <= 0.0) return 1.0;
	const double s = residual_abs / k;
	return 1.0 / (1.0 + s * s);
}

struct LinearizeAccum {
	Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
	Eigen::Matrix<double, 6, 1> b = Eigen::Matrix<double, 6, 1>::Zero();
	double error = 0.0;
	int inliers = 0;
	std::vector<double> inlier_dist;
};

bool LinearizePoint(const PointCloud& target,
                    const PointCloud& source,
                    const KdTree& tree,
                    const Eigen::Isometry3d& T,
                    std::size_t si,
                    const AlignSetting& setting,
                    double max_dist,
                    LinearizeAccum* acc) {
	const Eigen::Vector3d sp = source.points[si];
	const Eigen::Vector3d tp = T * sp;

	int ti = -1;
	double d2 = 0.0;
	if (!tree.Nearest(tp, &ti, &d2)) return false;
	if (d2 > max_dist * max_dist) return false;

	const Eigen::Vector3d q = target.points[ti];
	const Eigen::Vector3d r = q - tp;  // residual used by small_gicp

	if (setting.normal_check && target.has_normals() && source.has_normals()) {
		const Eigen::Vector3d ns = T.linear() * source.normals[si];
		if (target.normals[ti].dot(ns) < setting.min_normal_cos) return false;
	}

	Eigen::Matrix<double, 3, 6> J = Eigen::Matrix<double, 3, 6>::Zero();
	// small_gicp: J_rot = R [p]× , J_trans = -R
	J.block<3, 3>(0, 0) = T.linear() * Skew(sp);
	J.block<3, 3>(0, 3) = -T.linear();

	Eigen::Matrix3d info = Eigen::Matrix3d::Identity();
	double residual_for_kernel = std::sqrt(d2);
	double e = 0.0;

	if (setting.method == Method::GICP) {
		if (!target.has_covs() || !source.has_covs()) return false;
		const Eigen::Matrix3d RCR =
			target.covs[ti] + T.linear() * source.covs[si] * T.linear().transpose();
		Eigen::FullPivLU<Eigen::Matrix3d> lu(RCR);
		if (!lu.isInvertible()) return false;
		info = lu.inverse();
		const double mah2 = r.transpose() * info * r;
		if (!std::isfinite(mah2) || mah2 < 0.0) return false;
		residual_for_kernel = std::sqrt(std::max(0.0, mah2));
		e = 0.5 * mah2;
	} else if (setting.method == Method::PlaneICP) {
		if (!target.has_normals()) return false;
		const Eigen::Vector3d n = target.normals[ti];
		const double plane_r = n.dot(r);
		// Classic point-to-plane: scalar residual nᵀ r, Jacobian nᵀ J.
		const Eigen::Matrix<double, 1, 6> Jn = n.transpose() * J;
		double w = 1.0;
		if (setting.robust_kernel) w = CauchyWeight(std::abs(plane_r), setting.robust_k);
		acc->H += w * (Jn.transpose() * Jn);
		acc->b += w * (Jn.transpose() * plane_r);
		acc->error += w * 0.5 * plane_r * plane_r;
		acc->inliers += 1;
		acc->inlier_dist.push_back(std::sqrt(d2));
		return true;
	} else {
		e = 0.5 * d2;
	}

	double w = 1.0;
	if (setting.robust_kernel) {
		const double k = (setting.method == Method::GICP) ? setting.robust_k
		                                                 : std::max(setting.robust_k, 1e-3);
		w = CauchyWeight(residual_for_kernel, k);
	}

	acc->H += w * (J.transpose() * info * J);
	acc->b += w * (J.transpose() * info * r);
	acc->error += w * e;
	acc->inliers += 1;
	acc->inlier_dist.push_back(std::sqrt(d2));
	return true;
}

LinearizeAccum Linearize(const PointCloud& target,
                         const PointCloud& source,
                         const KdTree& tree,
                         const Eigen::Isometry3d& T,
                         const AlignSetting& setting,
                         double max_dist) {
	LinearizeAccum acc;
	acc.inlier_dist.reserve(source.size());
	for (std::size_t i = 0; i < source.size(); ++i) {
		LinearizePoint(target, source, tree, T, i, setting, max_dist, &acc);
	}
	return acc;
}

double Median(std::vector<double> v) {
	if (v.empty()) return 0.0;
	const auto mid = v.begin() + static_cast<int>(v.size() / 2);
	std::nth_element(v.begin(), mid, v.end());
	return *mid;
}

AlignResult AlignSingleScale(const PointCloud& target,
                             const PointCloud& source,
                             const Eigen::Isometry3d& init_T,
                             AlignSetting setting,
                             double voxel_hint) {
	AlignResult result;
	result.T = init_T;
	result.num_source = static_cast<int>(source.size());
	if (target.size() < 8 || source.size() < 8) return result;

	KdTree tree(target.points);
	if (setting.robust_k <= 0.0) {
		setting.robust_k = (setting.method == Method::GICP)
			? 1.0
			: std::max(0.02, 2.0 * voxel_hint);
	}
	double max_dist = std::max(setting.max_corr_dist, 1e-6);
	const double min_dist = (setting.min_corr_dist > 0.0)
		? setting.min_corr_dist
		: std::max(3.0 * voxel_hint, 1e-4);

	double lambda = setting.lm_lambda_init;

	for (int iter = 0; iter < setting.max_iterations; ++iter) {
		LinearizeAccum acc = Linearize(target, source, tree, result.T, setting, max_dist);
		result.iterations = iter + 1;
		result.num_inliers = acc.inliers;
		result.overlap = source.empty() ? 0.0
			: static_cast<double>(acc.inliers) / static_cast<double>(source.size());
		result.fitness = acc.inliers > 0 ? (acc.error / acc.inliers) : 0.0;
		result.final_max_dist = max_dist;

		if (acc.inliers < 6) {
			if (setting.verbose) {
				std::cerr << "  iter " << iter << " too few inliers (" << acc.inliers
				          << ") at max_dist=" << max_dist << "\n";
			}
			break;
		}

		if (setting.adaptive_max_dist && acc.inlier_dist.size() >= 8) {
			const double med = Median(acc.inlier_dist);
			const double shrunk = std::max(min_dist, setting.adaptive_scale * med);
			max_dist = std::min(max_dist, shrunk);
		}

		Eigen::Matrix<double, 6, 6> H = acc.H;
		H.diagonal().array() += lambda;
		const Eigen::Matrix<double, 6, 1> dx = H.ldlt().solve(-acc.b);
		if (!dx.allFinite()) break;

		const Eigen::Isometry3d T_new = Se3Exp(dx) * result.T;
		LinearizeAccum acc_new = Linearize(target, source, tree, T_new, setting, max_dist);
		if (acc_new.inliers >= 6 && acc_new.error < acc.error) {
			result.T = T_new;
			lambda = std::max(1e-8, lambda * 0.3);
			result.num_inliers = acc_new.inliers;
			result.overlap = static_cast<double>(acc_new.inliers) /
			                 static_cast<double>(source.size());
			result.fitness = acc_new.error / acc_new.inliers;
		} else {
			lambda = std::min(1e5, lambda * 4.0);
		}

		const double rot = dx.head<3>().norm();
		const double trans = dx.tail<3>().norm();
		if (setting.verbose) {
			std::cerr << "  iter " << iter << " " << MethodName(setting.method)
			          << " inliers=" << acc.inliers
			          << " max_d=" << max_dist
			          << " e=" << acc.error
			          << " drot=" << rot
			          << " dtrans=" << trans << "\n";
		}
		if (rot < setting.rotation_eps && trans < setting.translation_eps) {
			result.converged = true;
			break;
		}
	}

	result.success = result.overlap >= setting.min_inlier_ratio && result.num_inliers >= 6;
	return result;
}

PointCloud PrepareCloud(const PointCloud& cloud, double voxel, int knn, double clip) {
	PointCloud ds = VoxelDownsample(cloud, voxel);
	EstimateNormalsAndCovariances(ds, knn, clip);
	return ds;
}

}  // namespace

AlignResult Align(const PointCloud& target,
                  const PointCloud& source,
                  const Eigen::Isometry3d& init_T,
                  AlignSetting setting) {
	AlignResult result;
	result.T = init_T;
	if (target.empty() || source.empty()) return result;

	if (!setting.multiscale) {
		PointCloud tgt = PrepareCloud(target, setting.voxel, setting.cov_knn, setting.cov_clip_ratio);
		PointCloud src = PrepareCloud(source, setting.voxel, setting.cov_knn, setting.cov_clip_ratio);
		return AlignSingleScale(tgt, src, init_T, setting, setting.voxel);
	}

	const double voxels[3] = {setting.voxel * 4.0, setting.voxel * 2.0, setting.voxel};
	const double dist_scale[3] = {12.0, 8.0, 5.0};
	Eigen::Isometry3d T = init_T;
	for (int s = 0; s < 3; ++s) {
		const double v = voxels[s];
		AlignSetting st = setting;
		st.multiscale = false;
		st.max_iterations = (s == 2) ? setting.max_iterations : std::min(20, setting.max_iterations);
		// Coarse levels use a wider Euclidean gate so the basin is large;
		// the finest level is still capped by the user max_corr_dist.
		st.max_corr_dist = std::max(setting.max_corr_dist, dist_scale[s] * v);
		if (s == 2) st.max_corr_dist = setting.max_corr_dist;
		// P2P at the coarsest level is more stable when the initial gap is large;
		// GICP/P2L take over once the clouds are close.
		if (s == 0 && setting.method == Method::GICP) {
			st.method = Method::PlaneICP;
			st.robust_k = std::max(setting.robust_k, 3.0 * v);
		}
		PointCloud tgt = PrepareCloud(target, v, setting.cov_knn, setting.cov_clip_ratio);
		PointCloud src = PrepareCloud(source, v, setting.cov_knn, setting.cov_clip_ratio);
		if (setting.verbose) {
			std::cerr << "scale voxel=" << v << " max_dist=" << st.max_corr_dist
			          << " method=" << MethodName(st.method)
			          << " |tgt|=" << tgt.size() << " |src|=" << src.size() << "\n";
		}
		result = AlignSingleScale(tgt, src, T, st, v);
		T = result.T;
		if (!result.success && s == 0) {
			// Fall back to P2P if the first P2L scale failed to find inliers.
			st.method = Method::ICP;
			st.normal_check = false;
			result = AlignSingleScale(tgt, src, T, st, v);
			T = result.T;
		}
	}
	return result;
}

double CloudRMSE(const PointCloud& target,
                 const PointCloud& source,
                 const Eigen::Isometry3d& T,
                 double max_corr_dist) {
	if (target.empty() || source.empty()) return std::numeric_limits<double>::infinity();
	KdTree tree(target.points);
	const double max_d2 = max_corr_dist * max_corr_dist;
	double sum = 0.0;
	int n = 0;
	for (const auto& p : source.points) {
		int idx = -1;
		double d2 = 0.0;
		if (!tree.Nearest(T * p, &idx, &d2)) continue;
		if (d2 > max_d2) continue;
		sum += d2;
		++n;
	}
	if (n == 0) return std::numeric_limits<double>::infinity();
	return std::sqrt(sum / n);
}

}  // namespace small_reg
