#include "FitCircle3D.h"

#include <Eigen/Dense>

#include <cmath>
#include <iostream>
#include <string>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace {

int g_failed = 0;

void Expect(bool cond, const std::string& name)
{
    if (!cond) {
        std::cerr << "FAIL: " << name << "\n";
        ++g_failed;
    } else {
        std::cout << "PASS: " << name << "\n";
    }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr MakeSphere(const Eigen::Vector3f& center,
                                              float radius,
                                              int n_theta,
                                              int n_phi)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->reserve(static_cast<std::size_t>(n_theta * n_phi));
    for (int i = 0; i < n_theta; ++i) {
        const float theta = static_cast<float>(i) * static_cast<float>(M_PI)
            / static_cast<float>(n_theta - 1);
        for (int j = 0; j < n_phi; ++j) {
            const float phi = static_cast<float>(j) * 2.f * static_cast<float>(M_PI)
                / static_cast<float>(n_phi);
            pcl::PointXYZ p;
            p.x = center.x() + radius * std::sin(theta) * std::cos(phi);
            p.y = center.y() + radius * std::sin(theta) * std::sin(phi);
            p.z = center.z() + radius * std::cos(theta);
            cloud->push_back(p);
        }
    }
    return cloud;
}

} // namespace

int main()
{
    cv::Point3f center;
    float radius = 0.f;

    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr empty(new pcl::PointCloud<pcl::PointXYZ>);
        Expect(!fitCircle3D(empty, center, radius), "empty cloud fails");
        Expect(!fitCircle3D(nullptr, center, radius), "null cloud fails");
    }

    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr few(new pcl::PointCloud<pcl::PointXYZ>);
        few->push_back(pcl::PointXYZ(0, 0, 0));
        few->push_back(pcl::PointXYZ(1, 0, 0));
        few->push_back(pcl::PointXYZ(0, 1, 0));
        Expect(!fitCircle3D(few, center, radius), "three points fail");
    }

    {
        auto cloud = MakeSphere(Eigen::Vector3f(1.f, 2.f, 3.f), 10.f, 18, 36);
        Expect(fitCircle3D(cloud, center, radius), "sphere succeeds");
        Expect(std::fabs(center.x - 1.f) < 1e-2f
                   && std::fabs(center.y - 2.f) < 1e-2f
                   && std::fabs(center.z - 3.f) < 1e-2f,
               "sphere center");
        Expect(std::fabs(radius - 10.f) < 1e-2f, "sphere radius");
        std::cout << "  sphere center=(" << center.x << "," << center.y << "," << center.z
                  << ") radius=" << radius << "\n";
    }

    {
        auto cloud = MakeSphere(Eigen::Vector3f(5.f, -4.f, 8.f), 7.5f, 16, 32);
        cloud->push_back(pcl::PointXYZ(50.f, 50.f, 50.f));
        cloud->push_back(pcl::PointXYZ(-40.f, 30.f, -20.f));
        cloud->push_back(pcl::PointXYZ(0.f, 80.f, 1.f));
        Expect(fitCircle3D(cloud, center, radius), "outliers succeed");
        Expect(std::fabs(center.x - 5.f) < 0.15f
                   && std::fabs(center.y + 4.f) < 0.15f
                   && std::fabs(center.z - 8.f) < 0.15f,
               "outlier center");
        Expect(std::fabs(radius - 7.5f) < 0.15f, "outlier radius");
        std::cout << "  outlier center=(" << center.x << "," << center.y << "," << center.z
                  << ") radius=" << radius << "\n";
    }

    if (g_failed != 0) {
        std::cerr << g_failed << " test(s) failed\n";
        return 1;
    }
    std::cout << "all tests passed\n";
    return 0;
}
