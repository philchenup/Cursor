#ifndef GENG_RCIM2022_H_
#define GENG_RCIM2022_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

#include <Eigen/Dense>

#include <cstdint>
#include <vector>

/**
 * 复现 Geng et al., RCIM 2022 (79:102433)：
 * 改进 RANSAC 多平面拟合 → 相交平面求交线抽缝 → 等距轨迹 → 二面角焊枪姿态。
 * 长度单位与输入点云一致，工业扫描请使用 mm。
 */
class GengRcim2022
{
 public:
  struct Params
  {
    float voxel_leaf_mm = 2.0f;
    float plane_dist_mm = 1.5f;
    float local_sample_radius_mm = 12.0f;
    int ransac_iters = 250;
    int min_plane_inliers = 80;
    int max_planes = 8;
    float merge_normal_deg = 10.0f;
    float merge_offset_mm = 3.0f;
    float min_dihedral_deg = 50.0f;
    float max_dihedral_deg = 140.0f;
    float seam_band_mm = 8.0f;
    float min_seam_length_mm = 20.0f;
    int min_plane_support = 12;
    float trajectory_step_mm = 2.0f;
    std::uint32_t rng_seed = 42;
  };

  struct FittedPlane
  {
    Eigen::Vector3f normal = Eigen::Vector3f::UnitZ();
    float d = 0.0f;  // n·x + d = 0
    Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
    std::vector<int> inliers;
  };

  struct WeldSeam
  {
    Eigen::Vector3f start = Eigen::Vector3f::Zero();
    Eigen::Vector3f end = Eigen::Vector3f::Zero();
    Eigen::Vector3f line_dir = Eigen::Vector3f::UnitY();
    Eigen::Vector3f n1 = Eigen::Vector3f::UnitZ();
    Eigen::Vector3f n2 = Eigen::Vector3f::UnitX();
    Eigen::Vector3f torch_z = Eigen::Vector3f::UnitZ();
    float length_mm = 0.0f;
    pcl::PointCloud<pcl::PointXYZ> seam_cloud;
    pcl::PointCloud<pcl::PointNormal> trajectory;
  };

  void setParams(const Params& params) { params_ = params; }
  const Params& params() const { return params_; }

  void setInputCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud);
  bool compute();

  const std::vector<FittedPlane>& planes() const { return planes_; }
  const std::vector<WeldSeam>& seams() const { return seams_; }

  pcl::PointCloud<pcl::PointXYZ>::Ptr seamCloud() const;
  pcl::PointCloud<pcl::PointNormal>::Ptr trajectoryCloud() const;
  pcl::PointCloud<pcl::PointXYZ>::Ptr processedCloud() const { return cloud_; }

 private:
  void downsample();
  void extractPlanes();
  bool ransacOnePlane(const std::vector<int>& remaining, FittedPlane& plane) const;
  void refinePlane(FittedPlane& plane) const;
  void mergeSimilarPlanes();
  void extractSeamsAndTrajectories();
  bool buildSeam(const FittedPlane& a, const FittedPlane& b, WeldSeam& seam) const;
  void buildTrajectory(WeldSeam& seam) const;

  static bool fitThreePoints(const Eigen::Vector3f& p0,
                             const Eigen::Vector3f& p1,
                             const Eigen::Vector3f& p2,
                             Eigen::Vector3f& normal,
                             float& d);
  static float unsignedAngleDeg(const Eigen::Vector3f& a, const Eigen::Vector3f& b);
  static Eigen::Vector3f pointOnIntersection(const FittedPlane& a, const FittedPlane& b);
  static float pointToLineDistance(const Eigen::Vector3f& p,
                                   const Eigen::Vector3f& origin,
                                   const Eigen::Vector3f& dir);

  Params params_;
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr input_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_;
  std::vector<FittedPlane> planes_;
  std::vector<WeldSeam> seams_;
};

#endif  // GENG_RCIM2022_H_
