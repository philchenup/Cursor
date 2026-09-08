#ifndef FIT_CIRCLE_3D_H
#define FIT_CIRCLE_3D_H

#include <opencv2/core/types.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

/**
 * @brief 对三维点云拟合空间圆。
 *
 * 优先使用 PCL RANSAC（SACMODEL_CIRCLE3D）抗离群点；
 * 失败时回退到 PCA 平面投影 + 代数最小二乘圆拟合。
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
