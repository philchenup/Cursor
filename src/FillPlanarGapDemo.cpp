#include "FillPlanarGap.h"

#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

namespace {

pcl::PointCloud<pcl::PointXYZ>::Ptr loadXyz(const std::string& path)
{
    auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    std::ifstream input(path);
    float x = 0.f, y = 0.f, z = 0.f;
    while (input >> x >> y >> z)
        cloud->emplace_back(x, y, z);
    cloud->width = static_cast<std::uint32_t>(cloud->size());
    cloud->height = 1;
    cloud->is_dense = true;
    return cloud;
}

bool selfTest()
{
    auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    for (int y = 0; y < 50; ++y) {
        for (int x = 0; x < 80; ++x) {
            if (x >= 30 && x <= 44)
                continue;
            cloud->emplace_back(static_cast<float>(x), static_cast<float>(y), 0.f);
        }
    }
    const auto filled = fillPlanarGap(cloud, 1.f, 3.f);
    int inside = 0;
    for (const auto& point : *filled) {
        if (point.x <= 29.5f || point.x >= 44.5f || point.y < -0.5f || point.y > 49.5f)
            return false;
        if (std::fabs(point.z) > 1e-3f)
            return false;
        ++inside;
    }
    return inside > 600 && inside < 800;
}

} // namespace

int main(int argc, char** argv)
{
    if (!selfTest()) {
        std::cerr << "self-test failed\n";
        return 1;
    }
    if (argc < 3)
        return 0;

    const auto cloud = loadXyz(argv[1]);
    const auto filled = fillPlanarGap(cloud, 1.f, 3.f);
    std::ofstream output(argv[2]);
    for (const auto& point : *filled)
        output << point.x << " " << point.y << " " << point.z << "\n";
    std::cout << "filled " << filled->size() << "\n";
    return filled->empty() ? 1 : 0;
}
