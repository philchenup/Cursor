#ifndef WELD_SEAM_FINDER_H
#define WELD_SEAM_FINDER_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>

/**
 * @brief T 型板焊缝方向估计参数。输入点云坐标单位为毫米。
 */
struct WeldSeamParams {
    float planeDistance = 1.0f;     ///< 地面 RANSAC 内点距离阈值
    float heightThreshold = 20.0f;  ///< 距地面超过该高度的点视为平面外点并参与投影
    float zeroEpsilon = 1e-4f;      ///< 三点坐标都接近 0 时视为无效零点
    int sorMeanK = 30;              ///< 统计离群滤波的邻域点数
    float sorStddevMul = 1.0f;      ///< 统计离群滤波的标准差倍数
    float clusterTolerance = 5.0f;  ///< 欧式聚类容差
    int minClusterSize = 100;       ///< 小于该点数的点云团被删除
    float gridResolution = 2.0f;    ///< 空洞栅格边长，应大于点间距、小于缝宽
    float occupyRadius = 3.0f;      ///< 地面点占据半径，用于填平采样空隙
    float projectionBand = 8.0f;    ///< 平面外点投影的邻域宽度，用于锁定焊缝空洞
    int minHollowCells = 10;        ///< 小于该栅格数的空洞连通域被忽略
};

/**
 * @brief 焊缝方向估计结果。
 *
 * direction 为单位向量，位于地面平面内，是空洞区域 PCA 的第一主方向。
 * centroid 为空洞质心，与 direction 一起确定焊缝直线。
 * plane 为地面方程 ax + by + cz + d = 0，法向已单位化。
 */
struct WeldSeamResult {
    bool success = false;
    Eigen::Vector3f direction = Eigen::Vector3f::Zero();
    Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
    Eigen::Vector4f plane = Eigen::Vector4f::Zero();
    pcl::PointCloud<pcl::PointXYZ>::Ptr hollow;

    WeldSeamResult()
        : hollow(new pcl::PointCloud<pcl::PointXYZ>)
    {
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * @brief 估计 T 型板点云的焊缝主方向。
 *
 * 1. 去掉 NaN、零点、统计离群点以及小块点云团；
 * 2. 迭代平面分割，取内点最多的平面作为地面；
 * 3. 去掉到地面距离不超过 heightThreshold 的点；
 * 4. 将其余平面外点投影到地面；
 * 5. 在地面凸包内，取投影邻域中未被地面点覆盖的空洞；
 * 6. 对空洞区域做 PCA，第一主方向即为焊缝方向。
 *
 * 失败时 success 为 false，不抛出异常。
 */
WeldSeamResult FindWeldSeam(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
                            const WeldSeamParams& params = WeldSeamParams());

#endif // WELD_SEAM_FINDER_H
