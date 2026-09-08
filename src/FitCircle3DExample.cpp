#include "FitCircle3D.h"

#include <cmath>
#include <cstddef>
#include <iostream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    const float true_radius = 10.f;
    const cv::Point3f true_center(1.f, 2.f, 3.f);
    const int n_theta = 18;
    const int n_phi = 36;
    cloud->reserve(static_cast<std::size_t>(n_theta * n_phi));
    for (int i = 0; i < n_theta; ++i) {
        const float theta = static_cast<float>(i) * static_cast<float>(M_PI)
            / static_cast<float>(n_theta - 1);
        for (int j = 0; j < n_phi; ++j) {
            const float phi = static_cast<float>(j) * 2.f * static_cast<float>(M_PI)
                / static_cast<float>(n_phi);
            pcl::PointXYZ p;
            p.x = true_center.x + true_radius * std::sin(theta) * std::cos(phi);
            p.y = true_center.y + true_radius * std::sin(theta) * std::sin(phi);
            p.z = true_center.z + true_radius * std::cos(theta);
            cloud->push_back(p);
        }
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
