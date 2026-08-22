#pragma once

#include "registration/types.h"

namespace small_reg {

// Register source onto target (T * source ≈ target), following the
// small_gicp factor / LM layout. Optional coarse-to-fine voxel pyramid,
// adaptive Euclidean correspondence gate, Cauchy kernel, and normal-consistency
// rejection make GICP usable when max_corr_dist cannot be hand-tuned.
AlignResult Align(const PointCloud& target,
                  const PointCloud& source,
                  const Eigen::Isometry3d& init_T = Eigen::Isometry3d::Identity(),
                  AlignSetting setting = AlignSetting());

double CloudRMSE(const PointCloud& target,
                 const PointCloud& source,
                 const Eigen::Isometry3d& T,
                 double max_corr_dist);

}  // namespace small_reg
