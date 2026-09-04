/**
 * @file cloud.cpp
 * @author hjm (hjmalex@163.com)
 * @version 3.0
 * @date 2022-05-08
 */
#include "base/cloud.h"
#include "base/common.h"

#include <pcl/search/kdtree.h>

#include <cfloat>
#include <cmath>
#include <cstring>
#include <limits>
#include <vector>

namespace ct
{
    void Cloud::setCloudColor(const RGB& rgb)
    {
        const std::uint32_t rgb_ =
            (static_cast<std::uint32_t>(rgb.r) << 16 |
             static_cast<std::uint32_t>(rgb.g) << 8 |
             static_cast<std::uint32_t>(rgb.b));
        float rgb_f;
        std::memcpy(&rgb_f, &rgb_, sizeof(float));
        for (auto& i : points)
            i.rgb = rgb_f;
    }

    void Cloud::setCloudColor(const std::string& axis)
    {
        if (empty())
            return;

        float max_v = -FLT_MAX;
        float min_v = FLT_MAX;
        float fRed = 0.f, fGreen = 0.f, fBlue = 0.f;
        constexpr float range = 2.f / 3.f;
        constexpr uint8_t colorMax = std::numeric_limits<uint8_t>::max();

        auto colorize = [&](float value, float vmax, float vmin)
        {
            float hue = (vmax - value) / static_cast<float>(vmax - vmin);
            hue *= range;
            hue = range - hue;
            HSVtoRGB(hue, 1.f, 1.f, fRed, fGreen, fBlue);
        };

        if (axis == "x")
        {
            max_v = m_max.x;
            min_v = m_min.x;
            for (auto& i : points)
            {
                colorize(i.x, max_v, min_v);
                i.r = static_cast<uint8_t>(fRed * colorMax);
                i.g = static_cast<uint8_t>(fGreen * colorMax);
                i.b = static_cast<uint8_t>(fBlue * colorMax);
            }
        }
        else if (axis == "y")
        {
            max_v = m_max.y;
            min_v = m_min.y;
            for (auto& i : points)
            {
                colorize(i.y, max_v, min_v);
                i.r = static_cast<uint8_t>(fRed * colorMax);
                i.g = static_cast<uint8_t>(fGreen * colorMax);
                i.b = static_cast<uint8_t>(fBlue * colorMax);
            }
        }
        else if (axis == "z")
        {
            max_v = m_max.z;
            min_v = m_min.z;
            for (auto& i : points)
            {
                colorize(i.z, max_v, min_v);
                i.r = static_cast<uint8_t>(fRed * colorMax);
                i.g = static_cast<uint8_t>(fGreen * colorMax);
                i.b = static_cast<uint8_t>(fBlue * colorMax);
            }
        }
    }

    void Cloud::scale(double x, double y, double z, bool origin)
    {
        for (std::size_t j = 0; j < points.size(); ++j)
        {
            points[j].x = m_box.translation[0] + static_cast<float>(x) * (points[j].x - m_box.translation[0]);
            points[j].y = m_box.translation[1] + static_cast<float>(y) * (points[j].y - m_box.translation[1]);
            points[j].z = m_box.translation[2] + static_cast<float>(z) * (points[j].z - m_box.translation[2]);
        }
        if (origin)
        {
            Eigen::Affine3f trans;
            pcl::getTransformation(m_box.translation[0] * static_cast<float>(x) - m_box.translation[0],
                                   m_box.translation[1] * static_cast<float>(y) - m_box.translation[1],
                                   m_box.translation[2] * static_cast<float>(z) - m_box.translation[2],
                                   0, 0, 0, trans);
            pcl::transformPointCloud(*this, *this, trans);
        }
    }

    void Cloud::update(bool box_flag, bool type_flag, bool resolution_flag)
    {
        if (empty())
            return;

        if (type_flag)
        {
            const PointXYZRGBN& sample = points[rand() % size()];
            const bool no_normal = (sample.normal_x == 0.0f && sample.normal_y == 0.0f && sample.normal_z == 0.0f);
            const bool no_color = (sample.r == 0 && sample.g == 0 && sample.b == 0);
            if (no_normal)
                m_type = no_color ? CLOUD_TYPE_XYZ : CLOUD_TYPE_XYZRGB;
            else
                m_type = no_color ? CLOUD_TYPE_XYZN : CLOUD_TYPE_XYZRGBN;
        }

        if (box_flag)
        {
            pcl::getMinMax3D(*this, m_min, m_max);
            Eigen::Vector3f center = 0.5f * (m_min.getVector3fMap() + m_max.getVector3fMap());
            Eigen::Vector3f whd = m_max.getVector3fMap() - m_min.getVector3fMap();
            Eigen::Affine3f affine = m_box.pose;
            float roll = 0.f, pitch = 0.f, yaw = 0.f;
            pcl::getEulerAngles(affine, roll, pitch, yaw);
            affine = pcl::getTransformation(center[0], center[1], center[2], roll, pitch, yaw);
            m_box = {whd(0), whd(1), whd(2), affine, center, Eigen::Quaternionf(Eigen::Matrix3f::Identity())};
        }

        if (resolution_flag)
        {
            int n_points = 0;
            int nres = 0;
            const int num = size() < 1000 ? static_cast<int>(size()) : 1000;
            std::vector<int> indices(2);
            std::vector<float> sqr_distances(2);
            pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);
            tree->setInputCloud(this->makeShared());
            m_resolution = 0.0f;
            for (int i = 0; i < num; ++i)
            {
                const int index = rand() % static_cast<int>(size());
                if (!std::isfinite(points[index].x))
                    continue;
                nres = tree->nearestKSearch(index, 2, indices, sqr_distances);
                if (nres == 2)
                {
                    m_resolution += std::sqrt(sqr_distances[1]);
                    ++n_points;
                }
            }
            if (n_points != 0)
                m_resolution /= static_cast<float>(n_points);
        }
    }
}  // namespace ct
