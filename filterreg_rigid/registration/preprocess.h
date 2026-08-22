#pragma once

#include "registration/types.h"

namespace small_reg {

PointCloud VoxelDownsample(const PointCloud& cloud, double voxel);

// Estimate surface normals and planar covariances from k-NN. Regularizes the
// smallest eigenvalue to cov_clip_ratio * λ_max, matching small_gicp.
void EstimateNormalsAndCovariances(PointCloud& cloud, int knn, double cov_clip_ratio);

double MedianNearestNeighborDistance(const PointCloud& cloud, int sample_limit = 400);

}  // namespace small_reg
