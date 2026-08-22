#pragma once

#include <Eigen/Dense>
#include <vector>

namespace small_reg {

class KdTree {
public:
	KdTree() = default;
	explicit KdTree(const std::vector<Eigen::Vector3d>& pts);

	void Build(const std::vector<Eigen::Vector3d>& pts);

	bool Nearest(const Eigen::Vector3d& query, int* index, double* dist2) const;
	void Knn(const Eigen::Vector3d& query, int k,
	         std::vector<int>* indices, std::vector<double>* dist2) const;

	std::size_t size() const { return pts_.size(); }

private:
	struct Node {
		int point_index = -1;
		int axis = 0;
		int left = -1;
		int right = -1;
	};

	int BuildRange(int begin, int end, int depth);
	void NnVisit(int node, const Eigen::Vector3d& q, int* best, double* best_d2) const;
	void KnnVisit(int node, const Eigen::Vector3d& q, int k,
	              std::vector<std::pair<double, int>>* heap) const;

	std::vector<Eigen::Vector3d> pts_;
	std::vector<int> order_;
	std::vector<Node> nodes_;
	int root_ = -1;
};

}  // namespace small_reg
