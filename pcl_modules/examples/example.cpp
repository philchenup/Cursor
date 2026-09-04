/**
 * @file example.cpp
 * @brief 演示去掉 Qt 后的 PCL 模块用法：结果通过函数输出参数返回。
 */
#include "base/cloud.h"
#include "modules/features.h"
#include "modules/filters.h"
#include "modules/segmentation.h"
#include "modules/surface.h"

#include <pcl/common/io.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>

#include <cmath>
#include <iostream>
#include <vector>

namespace
{
    ct::Cloud::Ptr makeDemoCloud()
    {
        ct::Cloud::Ptr cloud(new ct::Cloud);
        cloud->setId("demo");

        // 平面簇
        for (int i = 0; i < 200; ++i)
        {
            for (int j = 0; j < 200; ++j)
            {
                ct::PointXYZRGBN p;
                p.x = static_cast<float>(i) * 0.01f;
                p.y = static_cast<float>(j) * 0.01f;
                p.z = 0.0f;
                p.r = 80;
                p.g = 160;
                p.b = 255;
                p.normal_x = 0.f;
                p.normal_y = 0.f;
                p.normal_z = 1.f;
                cloud->push_back(p);
            }
        }

        // 球面簇
        for (int i = 0; i < 80; ++i)
        {
            const float theta = static_cast<float>(i) * 0.08f;
            for (int j = 0; j < 80; ++j)
            {
                const float phi = static_cast<float>(j) * 0.08f;
                ct::PointXYZRGBN p;
                p.x = 3.0f + 0.4f * std::sin(theta) * std::cos(phi);
                p.y = 0.4f * std::sin(theta) * std::sin(phi);
                p.z = 0.4f * std::cos(theta);
                p.r = 255;
                p.g = 80;
                p.b = 80;
                p.normal_x = p.x - 3.0f;
                p.normal_y = p.y;
                p.normal_z = p.z;
                cloud->push_back(p);
            }
        }

        cloud->width = static_cast<std::uint32_t>(cloud->size());
        cloud->height = 1;
        cloud->is_dense = true;
        cloud->update();
        return cloud;
    }
}  // namespace

int main()
{
    ct::Cloud::Ptr cloud = makeDemoCloud();
    std::cout << "input cloud: id=" << cloud->id()
              << " size=" << cloud->size()
              << " type=" << cloud->type() << "\n";

    ct::Filters filters;
    filters.setInputCloud(cloud);
    ct::Cloud::Ptr filtered;
    float filter_time = 0.f;
    filters.VoxelGrid(0.05f, 0.05f, 0.05f, filtered, filter_time);
    std::cout << "VoxelGrid: size=" << filtered->size()
              << " time=" << filter_time << " ms\n";

    ct::Features features;
    features.setInputCloud(filtered);
    features.setKSearch(10);
    features.setRadiusSearch(0.0);
    ct::Cloud::Ptr with_normals;
    float normal_time = 0.f;
    features.NormalEstimation(0.f, 0.f, 0.f, with_normals, normal_time);
    std::cout << "NormalEstimation: size=" << with_normals->size()
              << " time=" << normal_time << " ms\n";

    ct::Box aabb = ct::Features::boundingBoxAABB(with_normals);
    std::cout << "AABB: " << aabb.width << " x " << aabb.height << " x " << aabb.depth << "\n";

    ct::Segmentation seg;
    seg.setInputCloud(with_normals);
    std::vector<ct::Cloud::Ptr> clusters;
    float seg_time = 0.f;
    seg.EuclideanClusterExtraction(0.15, 50, 100000, clusters, seg_time);
    std::cout << "EuclideanClusterExtraction: clusters=" << clusters.size()
              << " time=" << seg_time << " ms\n";

    ct::ModelCoefficients::Ptr coef;
    std::vector<ct::Cloud::Ptr> plane_clouds;
    float sac_time = 0.f;
    seg.SACSegmentation(pcl::SACMODEL_PLANE, pcl::SAC_RANSAC, 0.02, 200, 0.99,
                        true, 0.0, 0.0, plane_clouds, sac_time, coef);
    std::cout << "SACSegmentation: clusters=" << plane_clouds.size()
              << " coefs=" << (coef ? coef->values.size() : 0)
              << " time=" << sac_time << " ms\n";

    ct::Surface surface;
    surface.setInputCloud(with_normals);
    ct::PolygonMesh::Ptr mesh;
    float hull_time = 0.f;
    surface.ConvexHull(false, 3, mesh, hull_time);
    std::cout << "ConvexHull: polygons=" << (mesh ? mesh->polygons.size() : 0)
              << " time=" << hull_time << " ms\n";

    std::cout << "done.\n";
    return 0;
}
