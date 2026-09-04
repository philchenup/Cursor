/**
 * @file demo_segmentation.cpp
 * @brief segmentation.h 中每个公开接口的示例。
 */
#include "demo_utils.h"
#include "modules/segmentation.h"

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <cmath>
#include <iostream>
#include <vector>

namespace
{
    void logClusters(const char* name, const std::vector<ct::Cloud::Ptr>& clouds, float time)
    {
        std::cout << "[ " << name << " ] clusters=" << clouds.size();
        for (std::size_t i = 0; i < clouds.size() && i < 4; ++i)
            std::cout << " c" << i << "=" << (clouds[i] ? clouds[i]->size() : 0);
        std::cout << " time=" << time << " ms\n";
    }

    void demo_SACSegmentation()
    {
        ct::Segmentation seg;
        seg.setInputCloud(demo::makePlaneCloud());
        std::vector<ct::Cloud::Ptr> clouds;
        float time = 0.f;
        ct::ModelCoefficients::Ptr coef;
        seg.SACSegmentation(pcl::SACMODEL_PLANE, pcl::SAC_RANSAC, 0.02, 200, 0.99,
                            true, 0.0, 0.0, clouds, time, coef);
        std::cout << "[ SACSegmentation ] clusters=" << clouds.size()
                  << " coefs=" << (coef ? coef->values.size() : 0)
                  << " time=" << time << " ms\n";
    }

    void demo_SACSegmentationFromNormals()
    {
        ct::Segmentation seg;
        seg.setInputCloud(demo::makePlaneCloud());
        std::vector<ct::Cloud::Ptr> clouds;
        float time = 0.f;
        ct::ModelCoefficients::Ptr coef;
        seg.SACSegmentationFromNormals(pcl::SACMODEL_NORMAL_PLANE, pcl::SAC_RANSAC, 0.02, 200, 0.99,
                                       true, 0.0, 0.0, 0.1, 0.0, clouds, time, coef);
        std::cout << "[ SACSegmentationFromNormals ] clusters=" << clouds.size()
                  << " coefs=" << (coef ? coef->values.size() : 0)
                  << " time=" << time << " ms\n";
    }

    void demo_EuclideanClusterExtraction()
    {
        ct::Segmentation seg;
        seg.setInputCloud(demo::makeTwoClusterCloud());
        std::vector<ct::Cloud::Ptr> clouds;
        float time = 0.f;
        seg.EuclideanClusterExtraction(0.12, 20, 100000, clouds, time);
        logClusters("EuclideanClusterExtraction", clouds, time);
    }

    void demo_RegionGrowing()
    {
        ct::Segmentation seg;
        seg.setInputCloud(demo::makeTwoClusterCloud());
        std::vector<ct::Cloud::Ptr> clouds;
        float time = 0.f;
        seg.RegionGrowing(30, 100000, true, true, false, 5.0f, 0.05f, 1.0f, 20, clouds, time);
        logClusters("RegionGrowing", clouds, time);
    }

    void demo_RegionGrowingRGB()
    {
        ct::Segmentation seg;
        seg.setInputCloud(demo::makeTwoClusterCloud());
        std::vector<ct::Cloud::Ptr> clouds;
        float time = 0.f;
        seg.RegionGrowingRGB(30, 100000, true, true, false, 5.0f, 0.05f, 1.0f, 20,
                             15.f, 20.f, 0.1f, 10, clouds, time);
        logClusters("RegionGrowingRGB", clouds, time);
    }

    void demo_SupervoxelClustering()
    {
        ct::Segmentation seg;
        seg.setInputCloud(demo::makeTwoClusterCloud());
        std::vector<ct::Cloud::Ptr> clouds;
        float time = 0.f;
        seg.SupervoxelClustering(0.06f, 0.16f, 0.2f, 0.4f, 1.0f, false, clouds, time);
        logClusters("SupervoxelClustering", clouds, time);
    }

    void demo_ConditionalEuclideanClustering()
    {
        ct::ConditionFunction func = [](const ct::PointXYZRGBN& a, const ct::PointXYZRGBN& b, float) {
            return std::fabs(a.z - b.z) < 0.05f && std::fabs(static_cast<int>(a.r) - static_cast<int>(b.r)) < 40;
        };
        ct::Segmentation seg;
        seg.setInputCloud(demo::makeTwoClusterCloud());
        std::vector<ct::Cloud::Ptr> clouds;
        float time = 0.f;
        seg.ConditionalEuclideanClustering(func, 0.08f, 20, 100000, clouds, time);
        logClusters("ConditionalEuclideanClustering", clouds, time);
    }

    void demo_DonSegmentation()
    {
        ct::Segmentation seg;
        seg.setInputCloud(demo::makeSphereCloud(16, 16, 0.4f));
        std::vector<ct::Cloud::Ptr> clouds;
        float time = 0.f;
        seg.DonSegmentation(0.05, 1.0, 2.0, 0.1, 2.0, 10, 100000, clouds, time);
        logClusters("DonSegmentation", clouds, time);
    }

    void demo_ExtractPolygonalPrismData()
    {
        ct::Cloud::Ptr scene = demo::makePlaneCloud(24, 0.03f);
        // 在平面上方再加一些点
        for (int i = 0; i < 30; ++i)
            scene->push_back(demo::makePoint(0.2f, 0.2f, 0.05f + 0.002f * static_cast<float>(i), 255, 0, 0));
        scene->width = static_cast<std::uint32_t>(scene->size());
        scene->update();

        ct::Segmentation seg;
        seg.setInputCloud(scene);
        std::vector<ct::Cloud::Ptr> clouds;
        float time = 0.f;
        seg.ExtractPolygonalPrismData(demo::makeRectHull(0.05f, 0.05f, 0.5f, 0.5f),
                                      -0.02, 0.2f, 0.f, 0.f, 10.f, clouds, time);
        logClusters("ExtractPolygonalPrismData", clouds, time);
    }

    void demo_MinCutSegmentation()
    {
        ct::Cloud::Ptr cloud = demo::makeSphereCloud();
        cloud->update();
        ct::Segmentation seg;
        seg.setInputCloud(cloud);
        std::vector<ct::Cloud::Ptr> clouds;
        float time = 0.f;
        seg.MinCutSegmentation(0.25, 0.3, 0.8, 8, clouds, time);
        logClusters("MinCutSegmentation", clouds, time);
    }

    void demo_MorphologicalFilter()
    {
        ct::Segmentation seg;
        seg.setInputCloud(demo::makeTwoClusterCloud());
        std::vector<ct::Cloud::Ptr> clouds;
        float time = 0.f;
        seg.MorphologicalFilter(8, 1.0f, 0.3f, 0.15f, 0.08f, 2.0f, clouds, time);
        logClusters("MorphologicalFilter", clouds, time);
    }

    void demo_SeededHueSegmentation()
    {
        ct::Segmentation seg;
        seg.setInputCloud(demo::makeTwoClusterCloud());
        std::vector<ct::Cloud::Ptr> clouds;
        float time = 0.f;
        seg.SeededHueSegmentation(0.08, 12.f, clouds, time);
        logClusters("SeededHueSegmentation", clouds, time);
    }

    void demo_SegmentDifferences()
    {
        ct::Cloud::Ptr src = demo::makePlaneCloud(16, 0.04f);
        ct::Cloud::Ptr tgt = demo::makePlaneCloud(16, 0.04f);
        tgt->push_back(demo::makePoint(2.0f, 2.0f, 1.0f, 255, 0, 0));
        tgt->width = static_cast<std::uint32_t>(tgt->size());
        ct::Segmentation seg;
        seg.setInputCloud(src);
        std::vector<ct::Cloud::Ptr> clouds;
        float time = 0.f;
        seg.SegmentDifferences(tgt, 0.01, clouds, time);
        logClusters("SegmentDifferences", clouds, time);
    }
}  // namespace

int main()
{
    std::cout.setf(std::ios::unitbuf);
    std::cout << "======== demo_segmentation (segmentation.h) ========\n";
    demo_SACSegmentation();
    demo_SACSegmentationFromNormals();
    demo_EuclideanClusterExtraction();
    demo_RegionGrowing();
    demo_RegionGrowingRGB();
    demo_SupervoxelClustering();
    demo_ConditionalEuclideanClustering();
    demo_ExtractPolygonalPrismData();
    demo_MinCutSegmentation();
    demo_MorphologicalFilter();
    demo_SeededHueSegmentation();
    demo_SegmentDifferences();
    demo_DonSegmentation();
    std::cout << "done.\n";
    return 0;
}
