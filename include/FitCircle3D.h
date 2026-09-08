#ifndef FIT_CIRCLE_3D_H
#define FIT_CIRCLE_3D_H

#include <opencv2/core/types.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

/**
 * @brief 最小二乘拟合三维空间圆，输出圆心与半径。
 *
 * 对应 Python scipy.optimize.leastsq + spherrors：
 * 先用 PCL PCA 求点云平面并投影，再对平面内圆做非线性最小二乘，
 * 残差为 (x-a)^2 + (y-b)^2 - r^2。
 *
 * @param cloud  输入点云（至少 3 个有效点）
 * @param center 输出圆心
 * @param radius 输出半径
 * @return 拟合成功返回 true
 */
bool fitCircle3D(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                 cv::Point3f& center,
                 float& radius);

#endif // FIT_CIRCLE_3D_H
