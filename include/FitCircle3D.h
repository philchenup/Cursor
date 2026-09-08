#ifndef FIT_CIRCLE_3D_H
#define FIT_CIRCLE_3D_H

#include <opencv2/core/types.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

/**
 * @brief 对三维点云 RANSAC 拟合空间圆。
 *
 * 每次随机取 3 个点求外接圆（圆心在三点平面上），再用 PCL KdTree
 * 统计半径壳层内且贴近该平面的内点；保留内点最多的模型。
 *
 * @param cloud  输入点云（至少 3 个有效点）
 * @param center 输出圆心（世界坐标系）
 * @param radius 输出半径
 * @return 拟合成功返回 true，点云无效或拟合失败返回 false
 */
bool fitCircle3D(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                 cv::Point3f& center,
                 float& radius);

#endif // FIT_CIRCLE_3D_H
