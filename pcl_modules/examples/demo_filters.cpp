/**
 * @file demo_filters.cpp
 * @brief filters.h 中每个公开接口的示例。
 */
#include "demo_utils.h"
#include "modules/filters.h"

#include <pcl/ModelCoefficients.h>
#include <pcl/Vertices.h>
#include <pcl/sample_consensus/model_types.h>

#include <functional>
#include <iostream>
#include <vector>

namespace
{
    void runFilter(const char* name, const std::function<void(ct::Filters&, ct::Cloud::Ptr&, float&)>& fn,
                   const ct::Cloud::Ptr& input, bool negative = false)
    {
        ct::Filters filters;
        filters.setInputCloud(input);
        filters.setNegative(negative);
        ct::Cloud::Ptr out;
        float time = 0.f;
        fn(filters, out, time);
        demo::logCloud(name, out, time);
    }

    void demo_PassThrough()
    {
        runFilter("PassThrough", [](ct::Filters& f, ct::Cloud::Ptr& o, float& t) {
            f.PassThrough("z", -0.01f, 0.01f, o, t);
        }, demo::makePlaneCloud());
    }

    void demo_VoxelGrid()
    {
        runFilter("VoxelGrid", [](ct::Filters& f, ct::Cloud::Ptr& o, float& t) {
            f.VoxelGrid(0.06f, 0.06f, 0.06f, o, t);
        }, demo::makePlaneCloud());
    }

    void demo_ApproximateVoxelGrid()
    {
        runFilter("ApproximateVoxelGrid", [](ct::Filters& f, ct::Cloud::Ptr& o, float& t) {
            f.ApproximateVoxelGrid(0.06f, 0.06f, 0.06f, o, t);
        }, demo::makePlaneCloud());
    }

    void demo_StatisticalOutlierRemoval()
    {
        runFilter("StatisticalOutlierRemoval", [](ct::Filters& f, ct::Cloud::Ptr& o, float& t) {
            f.StatisticalOutlierRemoval(8, 1.0, o, t);
        }, demo::makeTwoClusterCloud());
    }

    void demo_RadiusOutlierRemoval()
    {
        runFilter("RadiusOutlierRemoval", [](ct::Filters& f, ct::Cloud::Ptr& o, float& t) {
            f.RadiusOutlierRemoval(0.08, 4, o, t);
        }, demo::makePlaneCloud(20, 0.04f));
    }

    void demo_ConditionalRemoval()
    {
        ct::ConditionAnd::Ptr cond(new ct::ConditionAnd);
        cond->addComparison(ct::FieldComparison::ConstPtr(
            new ct::FieldComparison("z", ct::CompareOp::GT, -0.01f)));
        runFilter("ConditionalRemoval", [&](ct::Filters& f, ct::Cloud::Ptr& o, float& t) {
            f.ConditionalRemoval(cond, o, t);
        }, demo::makePlaneCloud());
    }

    void demo_GridMinimum()
    {
        runFilter("GridMinimum", [](ct::Filters& f, ct::Cloud::Ptr& o, float& t) {
            f.GridMinimum(0.08f, o, t);
        }, demo::makePlaneCloud());
    }

    void demo_LocalMaximum()
    {
        runFilter("LocalMaximum", [](ct::Filters& f, ct::Cloud::Ptr& o, float& t) {
            f.LocalMaximum(0.08f, o, t);
        }, demo::makePlaneCloud());
    }

    void demo_ShadowPoints()
    {
        runFilter("ShadowPoints", [](ct::Filters& f, ct::Cloud::Ptr& o, float& t) {
            f.ShadowPoints(0.1f, o, t);
        }, demo::makePlaneCloud());
    }

    void demo_DownSampling()
    {
        runFilter("DownSampling", [](ct::Filters& f, ct::Cloud::Ptr& o, float& t) {
            f.DownSampling(0.06f, o, t);
        }, demo::makePlaneCloud());
    }

    void demo_UniformSampling()
    {
        runFilter("UniformSampling", [](ct::Filters& f, ct::Cloud::Ptr& o, float& t) {
            f.UniformSampling(0.06f, o, t);
        }, demo::makePlaneCloud());
    }

    void demo_RandomSampling()
    {
        runFilter("RandomSampling", [](ct::Filters& f, ct::Cloud::Ptr& o, float& t) {
            f.RandomSampling(200, 1, o, t);
        }, demo::makePlaneCloud());
    }

    void demo_ReSampling()
    {
        runFilter("ReSampling", [](ct::Filters& f, ct::Cloud::Ptr& o, float& t) {
            f.ReSampling(0.08f, 2, o, t);
        }, demo::makePlaneCloud(20, 0.04f));
    }

    void demo_SamplingSurfaceNormal()
    {
        runFilter("SamplingSurfaceNormal", [](ct::Filters& f, ct::Cloud::Ptr& o, float& t) {
            f.SamplingSurfaceNormal(20, 1, 0.5f, o, t);
        }, demo::makePlaneCloud());
    }

    void demo_NormalSpaceSampling()
    {
        runFilter("NormalSpaceSampling", [](ct::Filters& f, ct::Cloud::Ptr& o, float& t) {
            f.NormalSpaceSampling(200, 1, 4, o, t);
        }, demo::makeSphereCloud());
    }

    void demo_Convolution3D()
    {
        runFilter("Convolution3D", [](ct::Filters& f, ct::Cloud::Ptr& o, float& t) {
            f.Convolution3D(0.05f, 3.0f, 0.05f, 0.08, o, t);
        }, demo::makePlaneCloud(16, 0.04f));
    }

    void demo_CropBox()
    {
        runFilter("CropBox", [](ct::Filters& f, ct::Cloud::Ptr& o, float& t) {
            f.CropBox(Eigen::Vector4f(-0.1f, -0.1f, -0.1f, 1.f),
                      Eigen::Vector4f(0.4f, 0.4f, 0.1f, 1.f),
                      Eigen::Affine3f::Identity(), o, t);
        }, demo::makePlaneCloud());
    }

    void demo_CropHull()
    {
        const int n = 20;
        ct::Cloud::Ptr cloud = demo::makePlaneCloud(n, 0.04f);
        pcl::Vertices poly;
        // 取平面四角，避免前 4 个点共线导致 CropHull 崩溃
        poly.vertices = {
            0u,
            static_cast<std::uint32_t>(n - 1),
            static_cast<std::uint32_t>(n * n - 1),
            static_cast<std::uint32_t>(n * (n - 1))
        };
        std::vector<pcl::Vertices> polygons(1, poly);
        runFilter("CropHull", [&](ct::Filters& f, ct::Cloud::Ptr& o, float& t) {
            f.CropHull(polygons, 2, true, o, t);
        }, cloud);
    }

    void demo_FrustumCulling()
    {
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        pose(2, 3) = 1.0f;
        runFilter("FrustumCulling", [&](ct::Filters& f, ct::Cloud::Ptr& o, float& t) {
            f.FrustumCulling(pose, 60.f, 45.f, 0.1f, 5.0f, o, t);
        }, demo::makePlaneCloud());
    }

    void demo_MedianFilter()
    {
        runFilter("MedianFilter", [](ct::Filters& f, ct::Cloud::Ptr& o, float& t) {
            f.MedianFilter(3, 0.05f, o, t);
        }, demo::makeOrganizedCloud());
    }

    void demo_PlaneClipper3D()
    {
        runFilter("PlaneClipper3D", [](ct::Filters& f, ct::Cloud::Ptr& o, float& t) {
            f.PlaneClipper3D(Eigen::Vector4f(0.f, 0.f, 1.f, 0.01f), o, t);
        }, demo::makePlaneCloud());
    }

    void demo_ProjectInliers()
    {
        pcl::ModelCoefficients::Ptr model(new pcl::ModelCoefficients);
        model->values = {0.f, 0.f, 1.f, 0.f};
        runFilter("ProjectInliers", [&](ct::Filters& f, ct::Cloud::Ptr& o, float& t) {
            f.ProjectInliers(pcl::SACMODEL_PLANE, model, true, o, t);
        }, demo::makePlaneCloud(16, 0.04f, 0.02f));
    }
}  // namespace

int main()
{
    std::cout << "======== demo_filters (filters.h) ========\n";
    demo_PassThrough();
    demo_VoxelGrid();
    demo_ApproximateVoxelGrid();
    demo_StatisticalOutlierRemoval();
    demo_RadiusOutlierRemoval();
    demo_ConditionalRemoval();
    demo_GridMinimum();
    demo_LocalMaximum();
    demo_ShadowPoints();
    demo_DownSampling();
    demo_UniformSampling();
    demo_RandomSampling();
    demo_ReSampling();
    demo_SamplingSurfaceNormal();
    demo_NormalSpaceSampling();
    demo_Convolution3D();
    demo_CropBox();
    demo_CropHull();
    demo_FrustumCulling();
    demo_MedianFilter();
    demo_PlaneClipper3D();
    demo_ProjectInliers();
    std::cout << "done.\n";
    return 0;
}
