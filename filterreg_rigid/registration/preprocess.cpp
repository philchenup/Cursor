#include "registration/preprocess.h"

#include "registration/kdtree.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_map>

namespace small_reg {
namespace {

inline std::int64_t VoxelHash(int x, int y, int z) {
	return (static_cast<std::int64_t>(x) * 73856093ll) ^
	       (static_cast<std::int64_t>(y) * 19349663ll) ^
	       (static_cast<std::int64_t>(z) * 83492791ll);
}

}  // namespace

PointCloud VoxelDownsample(const PointCloud& cloud, double voxel) {
	PointCloud out;
	if (cloud.empty()) return out;
	if (voxel <= 0.0) return cloud;

	struct Acc {
		Eigen::Vector3d sum = Eigen::Vector3d::Zero();
		int count = 0;
	};
	std::unordered_map<std::int64_t, Acc> bins;
	bins.reserve(cloud.size());
	const double inv = 1.0 / voxel;
	for (const auto& p : cloud.points) {
		const int ix = static_cast<int>(std::floor(p.x() * inv));
		const int iy = static_cast<int>(std::floor(p.y() * inv));
		const int iz = static_cast<int>(std::floor(p.z() * inv));
		Acc& a = bins[VoxelHash(ix, iy, iz)];
		a.sum += p;
		++a.count;
	}
	out.points.reserve(bins.size());
	for (const auto& kv : bins) {
		out.points.push_back(kv.second.sum / static_cast<double>(kv.second.count));
	}
	return out;
}

void EstimateNormalsAndCovariances(PointCloud& cloud, int knn, double cov_clip_ratio) {
	cloud.normals.assign(cloud.size(), Eigen::Vector3d::UnitZ());
	cloud.covs.assign(cloud.size(), Eigen::Matrix3d::Identity() * 1e-3);
	if (cloud.size() < 3) return;

	knn = std::max(3, std::min(knn, static_cast<int>(cloud.size())));
	KdTree tree(cloud.points);
	const double clip = std::max(cov_clip_ratio, 1e-6);

	for (std::size_t i = 0; i < cloud.size(); ++i) {
		std::vector<int> idx;
		std::vector<double> d2;
		tree.Knn(cloud.points[i], knn, &idx, &d2);
		if (idx.size() < 3) continue;

		Eigen::Vector3d mean = Eigen::Vector3d::Zero();
		for (int j : idx) mean += cloud.points[j];
		mean /= static_cast<double>(idx.size());

		Eigen::Matrix3d C = Eigen::Matrix3d::Zero();
		for (int j : idx) {
			const Eigen::Vector3d d = cloud.points[j] - mean;
			C += d * d.transpose();
		}
		C /= std::max(1.0, static_cast<double>(idx.size() - 1));

		Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(C);
		if (eig.info() != Eigen::Success) continue;
		Eigen::Vector3d ev = eig.eigenvalues();  // ascending
		const double lam_max = std::max(ev(2), 1e-12);
		ev = ev.cwiseMax(lam_max * clip);
		C = eig.eigenvectors() * ev.asDiagonal() * eig.eigenvectors().transpose();

		Eigen::Vector3d n = eig.eigenvectors().col(0);
		if (n.norm() < 1e-12) n = Eigen::Vector3d::UnitZ();
		else n.normalize();
		// Flip toward the origin so neighboring patches agree more often.
		if (n.dot(mean) > 0.0) n = -n;

		cloud.covs[i] = C;
		cloud.normals[i] = n;
	}
}

double MedianNearestNeighborDistance(const PointCloud& cloud, int sample_limit) {
	if (cloud.size() < 2) return 0.0;
	KdTree tree(cloud.points);
	const int n = static_cast<int>(cloud.size());
	const int step = std::max(1, n / std::max(1, sample_limit));
	std::vector<double> dists;
	dists.reserve(static_cast<std::size_t>(sample_limit));
	for (int i = 0; i < n; i += step) {
		std::vector<int> idx;
		std::vector<double> d2;
		tree.Knn(cloud.points[i], 2, &idx, &d2);
		if (d2.size() >= 2) dists.push_back(std::sqrt(d2[1]));
	}
	if (dists.empty()) return 0.0;
	const auto mid = dists.begin() + static_cast<int>(dists.size() / 2);
	std::nth_element(dists.begin(), mid, dists.end());
	return *mid;
}

}  // namespace small_reg
