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

pcl::PointCloud<pcl::PointXYZ>::Ptr MakeCircle(const Eigen::Vector3f& center,
                                              const Eigen::Vector3f& normal,
                                              float radius,
                                              int samples,
                                              float arc_frac = 1.f)
{
    Eigen::Vector3f n = normal.normalized();
    Eigen::Vector3f u = n.unitOrthogonal();
    Eigen::Vector3f v = n.cross(u);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->reserve(static_cast<std::size_t>(samples));
    const float span = 2.f * static_cast<float>(M_PI) * arc_frac;
    for (int i = 0; i < samples; ++i) {
        const float theta = span * static_cast<float>(i) / static_cast<float>(samples);
        const Eigen::Vector3f p = center + radius * (std::cos(theta) * u + std::sin(theta) * v);
        cloud->push_back(pcl::PointXYZ(p.x(), p.y(), p.z()));
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
        Expect(!fitCircle3D(few, center, radius), "two points fail");
    }

    {
        auto cloud = MakeCircle(Eigen::Vector3f(1.f, 2.f, 3.f),
                                 Eigen::Vector3f(0.f, 0.f, 1.f),
                                 10.f, 72);
        Expect(fitCircle3D(cloud, center, radius), "xy-plane circle succeeds");
        Expect(std::fabs(center.x - 1.f) < 1e-3f
                   && std::fabs(center.y - 2.f) < 1e-3f
                   && std::fabs(center.z - 3.f) < 1e-3f,
               "xy-plane center");
        Expect(std::fabs(radius - 10.f) < 1e-3f, "xy-plane radius");
    }

    {
        auto cloud = MakeCircle(Eigen::Vector3f(5.f, -4.f, 8.f),
                                 Eigen::Vector3f(1.f, 1.f, 1.f),
                                 7.5f, 90);
        Expect(fitCircle3D(cloud, center, radius), "tilted circle succeeds");
        Expect(std::fabs(center.x - 5.f) < 5e-2f
                   && std::fabs(center.y + 4.f) < 5e-2f
                   && std::fabs(center.z - 8.f) < 5e-2f,
               "tilted center");
        Expect(std::fabs(radius - 7.5f) < 5e-2f, "tilted radius");
        std::cout << "  tilted center=(" << center.x << "," << center.y << "," << center.z
                  << ") radius=" << radius << "\n";
    }

    {
        auto cloud = MakeCircle(Eigen::Vector3f(0.f, 0.f, 0.f),
                                 Eigen::Vector3f(0.f, 1.f, 0.f),
                                 3.f, 50, 0.6f);
        Expect(fitCircle3D(cloud, center, radius), "partial arc succeeds");
        Expect(std::fabs(center.x) < 0.15f
                   && std::fabs(center.y) < 0.15f
                   && std::fabs(center.z) < 0.15f,
               "arc center");
        Expect(std::fabs(radius - 3.f) < 0.15f, "arc radius");
        std::cout << "  arc center=(" << center.x << "," << center.y << "," << center.z
                  << ") radius=" << radius << "\n";
    }

    {
        auto cloud = MakeCircle(Eigen::Vector3f(2.f, 3.f, 4.f),
                                 Eigen::Vector3f(0.f, 0.f, 1.f),
                                 5.f, 60);
        for (std::size_t i = 0; i < cloud->size(); ++i) {
            const float s = 0.02f * static_cast<float>((static_cast<int>(i) % 5) - 2);
            cloud->points[i].x += s;
            cloud->points[i].y += 0.5f * s;
        }
        Expect(fitCircle3D(cloud, center, radius), "noisy circle succeeds");
        Expect(std::fabs(center.x - 2.f) < 0.15f
                   && std::fabs(center.y - 3.f) < 0.15f
                   && std::fabs(center.z - 4.f) < 0.15f,
               "noisy center");
        Expect(std::fabs(radius - 5.f) < 0.15f, "noisy radius");
        std::cout << "  noisy center=(" << center.x << "," << center.y << "," << center.z
                  << ") radius=" << radius << "\n";
    }

    if (g_failed != 0) {
        std::cerr << g_failed << " test(s) failed\n";
        return 1;
    }
    std::cout << "all tests passed\n";
    return 0;
}
