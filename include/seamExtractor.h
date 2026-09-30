#pragma once
#ifndef SEAMEXTRACTOR_H_
#define SEAMEXTRACTOR_H_

#include "ComputeTwoPointPoses.h"
#include "base/cloud.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

#include <Eigen/Dense>

#include <cstdint>
#include <utility>
#include <vector>

/**
 * 从地面工件点云提取焊缝，并用 computeWeldTcpStartEnd 生成起终点 TCP。
 *
 * 平焊：Y 朝上定顺序，两端绕 Y 内倾 inward_deg。
 * 立焊：自下而上；起终点绕 X 向焊缝内倾 inward_deg。
 * 结果直接从 trajectoryCloud() 取。
 */
class SeamExtra
{
public:
    struct Params
    {
        float voxel_leaf_mm = 2.0f;
        float plane_dist_mm = 1.5f;
        float local_sample_radius_mm = 10.0f;
        int ransac_iters = 500;
        int min_plane_inliers = 3000;
        int max_planes = 4;
        float plane_cluster_tol_mm = 3.0f;
        float min_dihedral_deg = 50.0f;
        float max_dihedral_deg = 140.0f;
        float max_normal_dev_deg = 30.0f;
        float seam_band_mm = 8.0f;
        float min_seam_length_mm = 20.0f;
        int min_plane_support = 12;
        float trajectory_step_mm = 2.0f;
        /// 平焊绕 Y、立焊绕 X：起终点都向焊缝内倾。
        float inward_deg = 30.0f;
        std::uint32_t rng_seed = 42;
    };

    struct FittedPlane
    {
        Eigen::Vector3f normal = Eigen::Vector3f::UnitZ();
        float d = 0.0f;
        Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
        std::vector<int> inliers;
        ct::Cloud points;
    };

    struct WeldSeam
    {
        Eigen::Vector3f start = Eigen::Vector3f::Zero();
        Eigen::Vector3f end = Eigen::Vector3f::Zero();
        Eigen::Vector3f torch_z = Eigen::Vector3f::UnitZ();
        Eigen::Affine3f start_pose = Eigen::Affine3f::Identity();
        Eigen::Affine3f end_pose = Eigen::Affine3f::Identity();
        float length_mm = 0.0f;
        ct::Cloud seam_cloud;
        pcl::PointCloud<pcl::PointNormal> trajectory;
    };

    void setParams(const Params& params) { params_ = params; }
    const Params& params() const { return params_; }

    void setInputCloud(const ct::Cloud::Ptr& cloud);
    bool compute();

    const std::vector<FittedPlane>& planes() const { return planes_; }
    const std::vector<WeldSeam>& seams() const { return seams_; }

    /// 每条焊缝一对已修正 TCP：first = 起点，second = 终点。
    std::vector<std::pair<Eigen::Affine3f, Eigen::Affine3f>> trajectoryCloud() const;

    ct::Cloud::Ptr processedCloud() const { return cloud_; }
    ct::Cloud::Ptr segmentedCloud() const { return segmented_cloud_; }

private:
    void downsample();
    void estimateNormals();
    void extractPlanes();
    bool ransacOnePlane(const std::vector<int>& remaining, FittedPlane& plane) const;
    void refinePlane(FittedPlane& plane) const;
    void keepLargestCluster(FittedPlane& plane) const;
    void filterPlaneByNormal(FittedPlane& plane) const;
    bool nearOtherPlane(const Eigen::Vector3f& p, const FittedPlane& self, float tol) const;
    bool closerToOtherPlane(const Eigen::Vector3f& p,
        const FittedPlane& a,
        const FittedPlane& b) const;
    bool buildSeam(const FittedPlane& a, const FittedPlane& b, WeldSeam& seam) const;
    bool fillSeamPoses(WeldSeam& seam) const;

    static float planeDist(const Eigen::Vector3f& p, const FittedPlane& plane);
    static float lineDist(const Eigen::Vector3f& p,
        const Eigen::Vector3f& origin,
        const Eigen::Vector3f& dir);
    static float angleDeg(const Eigen::Vector3f& a, const Eigen::Vector3f& b);
    static Eigen::Vector3f intersectionPoint(const FittedPlane& a, const FittedPlane& b);

    Params params_;
    ct::Cloud::Ptr input_;
    ct::Cloud::Ptr cloud_;
    ct::Cloud::Ptr segmented_cloud_;
    pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree_;
    std::vector<FittedPlane> planes_;
    std::vector<WeldSeam> seams_;
    pcl::PointCloud<pcl::Normal>::Ptr normals_;
};

#endif
