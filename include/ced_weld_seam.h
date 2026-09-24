#ifndef CED_WELD_SEAM_H_
#define CED_WELD_SEAM_H_

#include <pcl/features/normal_3d.h>
#include <pcl/keypoints/keypoint.h>
#include <pcl/point_types.h>

namespace pcl
{
  /**
   * @brief 基于 CED 质心距离的焊缝点云检测器。
   *
   * 原始 CED（WACV 2023）用邻域质心偏移衡量几何显著性，因此会同时标出：
   *   - 单独一个点云面的自由边缘（邻域呈半盘，质心偏向面内）
   *   - 两个点云面相交处的焊缝 / 棱线（邻域跨越两个平面）
   *
   * 本检测器保留 CED 作为廉价预筛选，再对每个候选点做「双表面支撑」判定：
   *   1. 估计邻域法向，并聚成两个法向簇；
   *   2. 每个簇必须都能拟合出一个平面，且夹角落在焊缝二面角范围内；
   *   3. 查询点必须靠近两平面的交线。
   * 只有一个平面支撑的自由边会被丢弃，输出的是两面之间的焊缝点云
   * （默认不做非极大值抑制，因此结果是稠密缝带而不是稀疏关键点）。
   *
   * 参考：https://github.com/UCR-Robotics/CED_Detector
   *
   * @code
   * pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
   * pcl::PointCloud<pcl::PointXYZ>::Ptr seam (new pcl::PointCloud<pcl::PointXYZ>);
   * pcl::CEDWeldSeamDetector<pcl::PointXYZ, pcl::PointXYZ> detector;
   * detector.setRadiusSearch (0.03);
   * detector.setCentroidThreshold (0.10);
   * detector.setSupportRadius (0.04);
   * detector.setDihedralAngleRange (25.0, 155.0);
   * detector.setInputCloud (cloud);
   * detector.compute (*seam);
   * @endcode
   */
  template <typename PointInT, typename PointOutT>
  class CEDWeldSeamDetector : public Keypoint<PointInT, PointOutT>
  {
    public:
      using Ptr = shared_ptr<CEDWeldSeamDetector<PointInT, PointOutT> >;
      using ConstPtr = shared_ptr<const CEDWeldSeamDetector<PointInT, PointOutT> >;
      using NormalCloud = pcl::PointCloud<pcl::Normal>;
      using NormalCloudPtr = typename NormalCloud::Ptr;
      using NormalCloudConstPtr = typename NormalCloud::ConstPtr;

      using PointCloudIn = typename Keypoint<PointInT, PointOutT>::PointCloudIn;
      using PointCloudOut = typename Keypoint<PointInT, PointOutT>::PointCloudOut;

      using Keypoint<PointInT, PointOutT>::name_;
      using Keypoint<PointInT, PointOutT>::input_;
      using Keypoint<PointInT, PointOutT>::tree_;
      using Keypoint<PointInT, PointOutT>::search_radius_;
      using Keypoint<PointInT, PointOutT>::keypoints_indices_;

      CEDWeldSeamDetector ()
        : centroid_threshold_ (0.10)
        , min_neighbors_ (8)
        , support_radius_ (0.0)
        , normal_radius_ (0.0)
        , plane_distance_threshold_ (0.0)
        , seam_band_width_ (0.0)
        , cluster_gap_radius_ (0.0)
        , min_dihedral_deg_ (25.0)
        , max_dihedral_deg_ (155.0)
        , min_plane_inlier_ratio_ (0.22)
        , min_plane_inliers_ (8)
        , min_seam_cluster_size_ (12)
        , max_plane_rms_scale_ (1.8)
        , apply_non_max_suppression_ (false)
        , non_max_radius_ (0.0)
      {
        name_ = "CEDWeldSeamDetector";
      }

      ~CEDWeldSeamDetector () override = default;

      /** CED 显著性阈值：质心偏移 / 搜索半径，范围 [0,1]。略低于原版以保留焊缝点。 */
      inline void
      setCentroidThreshold (double centroid_threshold) { centroid_threshold_ = centroid_threshold; }

      inline void
      setMinNeighbors (int min_neighbors) { min_neighbors_ = min_neighbors; }

      /** 双平面分析半径。<=0 时默认使用 1.3 * search_radius。 */
      inline void
      setSupportRadius (double support_radius) { support_radius_ = support_radius; }

      /** 法向估计半径。<=0 时默认使用 0.6 * search_radius。 */
      inline void
      setNormalRadius (double normal_radius) { normal_radius_ = normal_radius; }

      /** 平面内点距离阈值。<=0 时默认使用 0.18 * search_radius。 */
      inline void
      setPlaneDistanceThreshold (double threshold) { plane_distance_threshold_ = threshold; }

      /** 允许偏离两平面交线的最大距离。<=0 时默认使用 0.35 * search_radius。 */
      inline void
      setSeamBandWidth (double width) { seam_band_width_ = width; }

      /** 焊缝连通簇的连接半径。<=0 时默认使用 search_radius。 */
      inline void
      setClusterGapRadius (double radius) { cluster_gap_radius_ = radius; }

      /** 两个支撑平面之间的二面角范围（度）。平行或近乎共面的薄板侧边会被滤掉。 */
      inline void
      setDihedralAngleRange (double min_deg, double max_deg)
      {
        min_dihedral_deg_ = min_deg;
        max_dihedral_deg_ = max_deg;
      }

      inline void
      setMinPlaneInlierRatio (double ratio) { min_plane_inlier_ratio_ = ratio; }

      inline void
      setMinPlaneInliers (int min_inliers) { min_plane_inliers_ = min_inliers; }

      /** 小于该点数的连通焊缝簇视为孤立误检并删除。 */
      inline void
      setMinSeamClusterSize (int min_size) { min_seam_cluster_size_ = min_size; }

      inline void
      setInputNormals (const NormalCloudConstPtr &normals) { input_normals_ = normals; }

      /**
       * @brief 是否对焊缝点再做非极大值抑制。
       * 默认关闭，输出稠密焊缝点云；打开后更接近原始 CED 的稀疏关键点。
       */
      inline void
      setNonMaxSuppression (bool enable, double non_max_radius = 0.0)
      {
        apply_non_max_suppression_ = enable;
        non_max_radius_ = non_max_radius;
      }

      /** 最近一次 compute() 得到的每个输入点 CED 质心距离。 */
      inline const std::vector<float> &
      getCentroidDistances () const { return (centroid_distances_); }

    protected:
      bool
      initCompute () override;

      void
      detectKeypoints (PointCloudOut &output) override;

      void
      estimateNormalsIfNeeded ();

      bool
      isTwoSurfaceJunction (int idx, const pcl::Indices &nn_indices) const;

      void
      removeSmallSeamClusters (std::vector<char> &is_seam) const;

      bool
      fitPcaPlane (const std::vector<int> &indices,
                   Eigen::Vector3f &centroid,
                   Eigen::Vector3f &normal,
                   float &rms) const;

      static float
      unsignedAngleDeg (const Eigen::Vector3f &a, const Eigen::Vector3f &b);

      static float
      distanceToIntersectionLine (const Eigen::Vector3f &query,
                                  const Eigen::Vector3f &c0,
                                  const Eigen::Vector3f &n0,
                                  const Eigen::Vector3f &c1,
                                  const Eigen::Vector3f &n1);

      double centroid_threshold_;
      int min_neighbors_;
      double support_radius_;
      double normal_radius_;
      double plane_distance_threshold_;
      double seam_band_width_;
      double cluster_gap_radius_;
      double min_dihedral_deg_;
      double max_dihedral_deg_;
      double min_plane_inlier_ratio_;
      int min_plane_inliers_;
      int min_seam_cluster_size_;
      double max_plane_rms_scale_;
      bool apply_non_max_suppression_;
      double non_max_radius_;

      NormalCloudConstPtr input_normals_;
      NormalCloudPtr computed_normals_;
      NormalCloudConstPtr normals_;
      std::vector<float> centroid_distances_;

      double resolved_support_radius_{0.0};
      double resolved_plane_distance_{0.0};
      double resolved_seam_band_{0.0};
      double resolved_cluster_gap_{0.0};
  };

}  // namespace pcl

#include "impl/ced_weld_seam.hpp"

#endif  // CED_WELD_SEAM_H_
