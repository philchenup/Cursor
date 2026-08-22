#include "registration/kdtree.h"

#include <algorithm>
#include <limits>

namespace small_reg {

KdTree::KdTree(const std::vector<Eigen::Vector3d>& pts) {
	Build(pts);
}

void KdTree::Build(const std::vector<Eigen::Vector3d>& pts) {
	pts_ = pts;
	nodes_.clear();
	order_.resize(pts_.size());
	for (int i = 0; i < static_cast<int>(pts_.size()); ++i) order_[i] = i;
	nodes_.reserve(pts_.size());
	root_ = BuildRange(0, static_cast<int>(order_.size()), 0);
}

int KdTree::BuildRange(int begin, int end, int depth) {
	if (begin >= end) return -1;
	const int axis = depth % 3;
	const int mid = begin + (end - begin) / 2;
	std::nth_element(order_.begin() + begin, order_.begin() + mid, order_.begin() + end,
	                 [this, axis](int a, int b) { return pts_[a](axis) < pts_[b](axis); });

	Node node;
	node.point_index = order_[mid];
	node.axis = axis;
	const int id = static_cast<int>(nodes_.size());
	nodes_.push_back(node);
	nodes_[id].left = BuildRange(begin, mid, depth + 1);
	nodes_[id].right = BuildRange(mid + 1, end, depth + 1);
	return id;
}

void KdTree::NnVisit(int node, const Eigen::Vector3d& q, int* best, double* best_d2) const {
	if (node < 0) return;
	const Node& n = nodes_[node];
	const double d2 = (pts_[n.point_index] - q).squaredNorm();
	if (d2 < *best_d2) {
		*best_d2 = d2;
		*best = n.point_index;
	}
	const double diff = q(n.axis) - pts_[n.point_index](n.axis);
	const int first = diff < 0.0 ? n.left : n.right;
	const int second = diff < 0.0 ? n.right : n.left;
	NnVisit(first, q, best, best_d2);
	if (diff * diff < *best_d2) NnVisit(second, q, best, best_d2);
}

bool KdTree::Nearest(const Eigen::Vector3d& query, int* index, double* dist2) const {
	if (root_ < 0) return false;
	int best = -1;
	double best_d2 = std::numeric_limits<double>::infinity();
	NnVisit(root_, query, &best, &best_d2);
	if (best < 0) return false;
	*index = best;
	*dist2 = best_d2;
	return true;
}

void KdTree::KnnVisit(int node, const Eigen::Vector3d& q, int k,
                      std::vector<std::pair<double, int>>* heap) const {
	if (node < 0) return;
	const Node& n = nodes_[node];
	const double d2 = (pts_[n.point_index] - q).squaredNorm();
	if (static_cast<int>(heap->size()) < k) {
		heap->emplace_back(d2, n.point_index);
		std::push_heap(heap->begin(), heap->end());
	} else if (d2 < heap->front().first) {
		std::pop_heap(heap->begin(), heap->end());
		heap->back() = {d2, n.point_index};
		std::push_heap(heap->begin(), heap->end());
	}

	const double diff = q(n.axis) - pts_[n.point_index](n.axis);
	const int first = diff < 0.0 ? n.left : n.right;
	const int second = diff < 0.0 ? n.right : n.left;
	KnnVisit(first, q, k, heap);
	const double worst = (static_cast<int>(heap->size()) < k)
		? std::numeric_limits<double>::infinity()
		: heap->front().first;
	if (diff * diff < worst) KnnVisit(second, q, k, heap);
}

void KdTree::Knn(const Eigen::Vector3d& query, int k,
                 std::vector<int>* indices, std::vector<double>* dist2) const {
	indices->clear();
	dist2->clear();
	if (root_ < 0 || k <= 0) return;
	k = std::min(k, static_cast<int>(pts_.size()));
	std::vector<std::pair<double, int>> heap;
	heap.reserve(static_cast<std::size_t>(k));
	KnnVisit(root_, query, k, &heap);
	std::sort_heap(heap.begin(), heap.end());
	indices->reserve(heap.size());
	dist2->reserve(heap.size());
	for (const auto& p : heap) {
		dist2->push_back(p.first);
		indices->push_back(p.second);
	}
}

}  // namespace small_reg
