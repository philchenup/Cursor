#pragma once

#include <pcl/visualization/pcl_visualizer.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

// PCL 1.15 删除了 PCLVisualizer::updateCamera()。
// setCameraPosition 会写入 vtkCamera 并调用 Render，不再需要那一次调用。
// 如果原来单独写了 viewer.updateCamera() / viewer->updateCamera() 只为刷新窗口，
// 改成 viewer.spinOnce(1, true)。
inline void setViewerPose(pcl::visualization::PCLVisualizer& viewer,
                          const Eigen::Affine3f& viewer_pose)
{
    const Eigen::Vector3f position = viewer_pose.translation();
    const Eigen::Vector3f look_at =
        viewer_pose.rotation() * Eigen::Vector3f(0.f, 0.f, 1.f) + position;
    const Eigen::Vector3f up =
        viewer_pose.rotation() * Eigen::Vector3f(0.f, -1.f, 0.f);

    viewer.setCameraPosition(
        static_cast<double>(position.x()),
        static_cast<double>(position.y()),
        static_cast<double>(position.z()),
        static_cast<double>(look_at.x()),
        static_cast<double>(look_at.y()),
        static_cast<double>(look_at.z()),
        static_cast<double>(up.x()),
        static_cast<double>(up.y()),
        static_cast<double>(up.z()));
}
