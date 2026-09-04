#ifndef CT_DEMO_UTILS_H
#define CT_DEMO_UTILS_H

#include "base/cloud.h"

#include <cmath>
#include <cstdint>
#include <iostream>
#include <string>

namespace demo
{
    inline void log(const std::string& name, const std::string& msg)
    {
        std::cout << "[ " << name << " ] " << msg << std::endl;
    }

    inline void logCloud(const std::string& name, const ct::Cloud::Ptr& cloud, float time_ms = -1.f)
    {
        std::cout << "[ " << name << " ] size=" << (cloud ? cloud->size() : 0);
        if (cloud)
            std::cout << " id=" << cloud->id();
        if (time_ms >= 0.f)
            std::cout << " time=" << time_ms << " ms";
        std::cout << std::endl;
    }

    inline ct::PointXYZRGBN makePoint(float x, float y, float z,
                                      std::uint8_t r = 128, std::uint8_t g = 128, std::uint8_t b = 128,
                                      float nx = 0.f, float ny = 0.f, float nz = 1.f)
    {
        ct::PointXYZRGBN p;
        p.x = x;
        p.y = y;
        p.z = z;
        p.r = r;
        p.g = g;
        p.b = b;
        p.normal_x = nx;
        p.normal_y = ny;
        p.normal_z = nz;
        p.curvature = 0.f;
        return p;
    }

    /** 带颜色和法线的平面点云（默认 xy 平面）。 */
    inline ct::Cloud::Ptr makePlaneCloud(int n = 40, float step = 0.03f, float z = 0.f)
    {
        ct::Cloud::Ptr cloud(new ct::Cloud);
        cloud->setId("plane");
        for (int i = 0; i < n; ++i)
        {
            for (int j = 0; j < n; ++j)
            {
                const float x = static_cast<float>(i) * step;
                const float y = static_cast<float>(j) * step;
                cloud->push_back(makePoint(x, y, z, 80, 160, 255, 0.f, 0.f, 1.f));
            }
        }
        cloud->width = static_cast<std::uint32_t>(cloud->size());
        cloud->height = 1;
        cloud->is_dense = true;
        cloud->update();
        return cloud;
    }

    /** 有组织（行列）点云，供中值滤波等使用。 */
    inline ct::Cloud::Ptr makeOrganizedCloud(int width = 32, int height = 32, float step = 0.03f)
    {
        ct::Cloud::Ptr cloud(new ct::Cloud);
        cloud->setId("organized");
        cloud->width = static_cast<std::uint32_t>(width);
        cloud->height = static_cast<std::uint32_t>(height);
        cloud->is_dense = true;
        cloud->points.resize(static_cast<std::size_t>(width * height));
        for (int r = 0; r < height; ++r)
        {
            for (int c = 0; c < width; ++c)
            {
                auto& p = cloud->points[static_cast<std::size_t>(r * width + c)];
                p = makePoint(static_cast<float>(c) * step, static_cast<float>(r) * step,
                              0.01f * static_cast<float>((r + c) % 3),
                              200, 80, 80, 0.f, 0.f, 1.f);
            }
        }
        cloud->update();
        return cloud;
    }

    /** 球面点云。 */
    inline ct::Cloud::Ptr makeSphereCloud(int n_theta = 24, int n_phi = 24, float radius = 0.4f)
    {
        ct::Cloud::Ptr cloud(new ct::Cloud);
        cloud->setId("sphere");
        for (int i = 1; i < n_theta; ++i)  // 跳过极点，避免重复点导致特征估计崩溃
        {
            const float theta = static_cast<float>(i) * (3.1415926f / static_cast<float>(n_theta));
            for (int j = 0; j < n_phi; ++j)
            {
                const float phi = static_cast<float>(j) * (6.2831853f / static_cast<float>(n_phi));
                const float nx = std::sin(theta) * std::cos(phi);
                const float ny = std::sin(theta) * std::sin(phi);
                const float nz = std::cos(theta);
                cloud->push_back(makePoint(radius * nx, radius * ny, radius * nz,
                                           255, 80, 80, nx, ny, nz));
            }
        }
        cloud->width = static_cast<std::uint32_t>(cloud->size());
        cloud->height = 1;
        cloud->is_dense = true;
        cloud->update();
        return cloud;
    }

    /** 两个分离簇：平面 + 球面。 */
    inline ct::Cloud::Ptr makeTwoClusterCloud()
    {
        ct::Cloud::Ptr cloud = makePlaneCloud(30, 0.03f);
        ct::Cloud::Ptr sphere = makeSphereCloud(16, 16, 0.25f);
        for (const auto& p : sphere->points)
        {
            ct::PointXYZRGBN q = p;
            q.x += 2.0f;
            cloud->push_back(q);
        }
        cloud->setId("two_clusters");
        cloud->width = static_cast<std::uint32_t>(cloud->size());
        cloud->height = 1;
        cloud->update();
        return cloud;
    }

    /** 少量带法线的点，供 PFH/SHOT 等局部描述子使用。 */
    inline ct::Cloud::Ptr makeTinyFeatureCloud(int n = 40)
    {
        ct::Cloud::Ptr cloud(new ct::Cloud);
        cloud->setId("tiny_feature");
        for (int i = 0; i < n; ++i)
        {
            const float a = static_cast<float>(i) * 0.15f;
            const float x = 0.3f * std::cos(a);
            const float y = 0.3f * std::sin(a);
            const float z = 0.02f * static_cast<float>(i);
            cloud->push_back(makePoint(x, y, z, 200, 80, 40, 0.f, 0.f, 1.f));
        }
        cloud->width = static_cast<std::uint32_t>(cloud->size());
        cloud->height = 1;
        cloud->is_dense = true;
        cloud->update();
        return cloud;
    }

    /** 平面矩形凸包点，供棱柱提取 / CropHull。 */
    inline ct::Cloud::Ptr makeRectHull(float x0 = 0.f, float y0 = 0.f, float x1 = 0.6f, float y1 = 0.6f)
    {
        ct::Cloud::Ptr hull(new ct::Cloud);
        hull->setId("hull");
        hull->push_back(makePoint(x0, y0, 0.f));
        hull->push_back(makePoint(x1, y0, 0.f));
        hull->push_back(makePoint(x1, y1, 0.f));
        hull->push_back(makePoint(x0, y1, 0.f));
        hull->width = 4;
        hull->height = 1;
        hull->is_dense = true;
        hull->update();
        return hull;
    }
}  // namespace demo

#endif  // CT_DEMO_UTILS_H
