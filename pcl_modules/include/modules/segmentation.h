/**
 * @file segmentation.h
 * @author hjm (hjmalex@163.com)
 * @version 3.0
 * @date 2022-05-10
 */
#ifndef CT_MODULES_SEGMENTATION_H
#define CT_MODULES_SEGMENTATION_H

#include "base/cloud.h"
#include "base/exports.h"

#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>

#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace ct
{
    typedef pcl::PointNormal                                    PointN;
    typedef pcl::PointIndices                                   PointIndices;
    typedef pcl::PointIndicesPtr                                PointIndicesPtr;
    typedef pcl::ModelCoefficients                              ModelCoefficients;
    typedef std::vector<PointIndices>                           IndicesClusters;
    typedef std::shared_ptr<std::vector<pcl::PointIndices>>     IndicesClustersPtr;
    typedef std::function<bool(const PointXYZRGBN&, const PointXYZRGBN&, float)> ConditionFunction;

    class CT_EXPORT Segmentation
    {
    public:
        Segmentation() : cloud_(nullptr), negative_(false) {}

        /**
         * @brief 设置输入点云
         */
        void setInputCloud(const Cloud::Ptr& cloud) { cloud_ = cloud; }

        /**
         * @brief 设置是应用点过滤的常规条件，还是应用倒置条件
         */
        void setNegative(bool negative) { negative_ = negative; }

        /**
         * @brief 表示 Sample Consensus 方法和模型的分割类
         * @param clouds 分割得到的点云簇
         * @param time 耗时（毫秒）
         * @param cofe 模型系数
         */
        void SACSegmentation(int model, int method, double threshold, int max_iterations, double probability,
                             bool optimize, double min_radius, double max_radius,
                             std::vector<Cloud::Ptr>& clouds, float& time, ModelCoefficients::Ptr& cofe);

        /**
         * @brief 基于法线的 Sample Consensus 分割
         */
        void SACSegmentationFromNormals(int model, int method, double threshold, int max_iterations, double probability,
                                        bool optimize, double min_radius, double max_radius, double distance_weight, double d,
                                        std::vector<Cloud::Ptr>& clouds, float& time, ModelCoefficients::Ptr& cofe);

        /**
         * @brief 表示用于欧几里得意义上的聚类提取的分割类
         */
        void EuclideanClusterExtraction(double tolerance, int min_cluster_size, int max_cluster_size,
                                        std::vector<Cloud::Ptr>& clouds, float& time);

        /**
         * @brief 实现用于分割的众所周知的区域增长算法
         */
        void RegionGrowing(int min_cluster_size, int max_cluster_size, bool smooth_mode, bool curvature_test,
                           bool residual_test, float smoothness_threshold, float residual_threshold,
                           float curvature_threshold, int neighbours,
                           std::vector<Cloud::Ptr>& clouds, float& time);

        /**
         * @brief 实现用于基于点颜色进行分割的区域增长算法
         */
        void RegionGrowingRGB(int min_cluster_size, int max_cluster_size, bool smooth_mode, bool curvature_test,
                              bool residual_test, float smoothness_threshold, float residual_threshold,
                              float curvature_threshold, int neighbours, float pt_thresh, float re_thresh,
                              float dis_thresh, int nghbr_number,
                              std::vector<Cloud::Ptr>& clouds, float& time);

        /**
         * @brief 实现基于体素结构、法线和 rgb 值的超体素算法。
         */
        void SupervoxelClustering(float voxel_resolution, float seed_resolution, float color_importance,
                                  float spatial_importance, float normal_importance, bool camera_transform,
                                  std::vector<Cloud::Ptr>& clouds, float& time);

        /**
         * @brief 基于欧几里得距离和用户定义的聚类执行分割
         */
        void ConditionalEuclideanClustering(ConditionFunction func, float cluster_tolerance,
                                            int min_cluster_size, int max_cluster_size,
                                            std::vector<Cloud::Ptr>& clouds, float& time);

        /**
         * @brief 法线差分割
         */
        void DonSegmentation(double mean_radius, double scale1, double scale2, double threshold,
                             double segradius, int minClusterSize, int maxClusterSize,
                             std::vector<Cloud::Ptr>& clouds, float& time);

        /**
         * @brief 使用一组表示平面模型的点索引，并与给定的高度一起生成 3D 多边形棱柱
         */
        void ExtractPolygonalPrismData(const Cloud::Ptr& hull, double height_min, double height_max,
                                       float vpx, float vpy, float vpz,
                                       std::vector<Cloud::Ptr>& clouds, float& time);

        /**
         * @brief 最小割分割
         */
        void MinCutSegmentation(double sigma, double radius, double weight, int neighbour_number,
                                std::vector<Cloud::Ptr>& clouds, float& time);

        /**
         * @brief 渐进形态学滤波
         */
        void MorphologicalFilter(int max_window_size, float slope, float max_distance, float initial_distance,
                                 float cell_size, float base,
                                 std::vector<Cloud::Ptr>& clouds, float& time);

        /**
         * @brief 种子色调分割
         */
        void SeededHueSegmentation(double tolerance, float delta_hue,
                                   std::vector<Cloud::Ptr>& clouds, float& time);

        /**
         * @brief 获得两个空间对齐的点云之间的差异
         */
        void SegmentDifferences(const Cloud::Ptr& tar_cloud, double sqr_threshold,
                                std::vector<Cloud::Ptr>& clouds, float& time);

    private:
        Cloud::Ptr cloud_;
        bool negative_;

        std::vector<Cloud::Ptr> getClusters(const IndicesClustersPtr& clusters);
        std::vector<Cloud::Ptr> getClusters(const PointIndicesPtr& clusters);
    };
}  // namespace ct

#endif  // CT_MODULES_SEGMENTATION_H
