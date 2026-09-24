// CED-3D is adapted from UCR-Robotics/CED_Detector (MIT License):
// Copyright (c) 2022 Autonomous Robots and Control Systems (ARCS) Lab
#ifndef CED_3D_H_
#define CED_3D_H_

#include <pcl/keypoints/keypoint.h>

namespace pcl
{
  /**
   * @brief CEDKeypoint3D 是 CED 检测器的纯几何版本，不需要法向或特征值分解。
   *
   * 来源（MIT License）：
   * H. Teng, D. Chatziparaschis, X. Kan, A. K. Roy-Chowdhury and K. Karydis,
   * "Centroid Distance Keypoint Detector for Colored Point Clouds", WACV 2023.
   * https://github.com/UCR-Robotics/CED_Detector
   *
   * 质心距离会同时响应两类区域：
   *   1. 单独点云的自由边缘（邻域被截断，质心偏向面内）
   *   2. 两个点云面相交形成的焊缝 / 棱线
   * 若只需要焊缝，请使用 CEDWeldSeamDetector。
   */
  template <typename PointInT, typename PointOutT>
  class CEDKeypoint3D : public Keypoint<PointInT, PointOutT>
  {
    public:
      using Ptr = shared_ptr<CEDKeypoint3D<PointInT, PointOutT> >;
      using ConstPtr = shared_ptr<const CEDKeypoint3D<PointInT, PointOutT> >;

      using PointCloudIn = typename Keypoint<PointInT, PointOutT>::PointCloudIn;
      using PointCloudOut = typename Keypoint<PointInT, PointOutT>::PointCloudOut;

      using Keypoint<PointInT, PointOutT>::name_;
      using Keypoint<PointInT, PointOutT>::input_;
      using Keypoint<PointInT, PointOutT>::tree_;
      using Keypoint<PointInT, PointOutT>::search_radius_;
      using Keypoint<PointInT, PointOutT>::keypoints_indices_;

      CEDKeypoint3D ()
        : centroid_threshold_ (0.4)
        , non_max_radius_ (0.0)
        , min_neighbors_ (5)
      {
        name_ = "CEDKeypoint3D";
        non_max_radius_ = search_radius_;
      }

      ~CEDKeypoint3D () override = default;

      inline void
      setCentroidThreshold (double centroid_threshold) { centroid_threshold_ = centroid_threshold; }

      inline void
      setNonMaxRadius (double non_max_radius) { non_max_radius_ = non_max_radius; }

      inline void
      setMinNeighbors (int min_neighbors) { min_neighbors_ = min_neighbors; }

    protected:
      bool
      initCompute () override;

      void
      detectKeypoints (PointCloudOut &output) override;

      double centroid_threshold_;
      double non_max_radius_;
      int min_neighbors_;
  };

}  // namespace pcl

#include "impl/ced_3d.hpp"

#endif  // CED_3D_H_
