#ifndef FIT_CIRCLE_3D_H
#define FIT_CIRCLE_3D_H

#include <opencv2/core/types.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

/**
 * @brief RANSAC 拟合空间球/圆，输出球心（圆心）与半径。
 *
 * 直接对应 Python ransac_fit_sphere_process：每次随机 4 点解 3x3 线性方程，
 * 用 PCL KdTree 统计半径壳层内点数，保留内点最多的模型。
 *
 * @param cloud  输入点云（至少 4 个点）
 * @param center 输出球心
 * @param radius 输出半径
 * @return 拟合成功返回 true
 */
bool fitCircle3D(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                 cv::Point3f& center,
                 float& radius);

#endif // FIT_CIRCLE_3D_H
