#include "FitCircle3D.h"

#include <cmath>
#include <cstddef>
#include <iostream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// 用法示例：对点云拟合空间圆，得到圆心和半径
bool FitCircleFromCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                        cv::Point3f& center,
                        float& radius)
{
    if (!fitCircle3D(cloud, center, radius)) {
        return false;
    }
    return true;
}

int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    const float true_radius = 10.f;
    const cv::Point3f true_center(1.f, 2.f, 3.f);
    const int samples = 72;
    cloud->reserve(static_cast<std::size_t>(samples));
    for (int i = 0; i < samples; ++i) {
        const float theta = static_cast<float>(i) * 2.f * static_cast<float>(M_PI)
            / static_cast<float>(samples);
        pcl::PointXYZ p;
        p.x = true_center.x + true_radius * std::cos(theta);
        p.y = true_center.y + true_radius * std::sin(theta);
        p.z = true_center.z;
        cloud->push_back(p);
    }

    cv::Point3f center;
    float radius = 0.f;
    if (!fitCircle3D(cloud, center, radius)) {
        std::cerr << "fitCircle3D failed\n";
        return 1;
    }

    std::cout << "center = (" << center.x << ", " << center.y << ", " << center.z << ")\n"
              << "radius = " << radius << "\n";
    return 0;
}
