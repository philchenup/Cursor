/**
 * @file demo_surface.cpp
 * @brief surface.h 中每个公开接口的示例。
 */
#include "demo_utils.h"
#include "modules/surface.h"

#include <iostream>

namespace
{
    void logMesh(const char* name, const ct::PolygonMesh::Ptr& mesh, float time)
    {
        const std::size_t n_poly = mesh ? mesh->polygons.size() : 0;
        std::cout << "[ " << name << " ] polygons=" << n_poly << " time=" << time << " ms\n";
    }

    void demo_GreedyProjectionTriangulation()
    {
        ct::Surface sur;
        sur.setInputCloud(demo::makeSphereCloud(16, 16, 0.4f));
        ct::PolygonMesh::Ptr mesh;
        float time = 0.f;
        sur.GreedyProjectionTriangulation(2.5, 50, 0.12, 10.0, 120.0, 45.0, false, false, mesh, time);
        logMesh("GreedyProjectionTriangulation", mesh, time);
    }

    void demo_GridProjection()
    {
        ct::Surface sur;
        sur.setInputCloud(demo::makeSphereCloud(12, 12, 0.3f));
        ct::PolygonMesh::Ptr mesh;
        float time = 0.f;
        sur.GridProjection(0.08, 1, 10, 5, mesh, time);
        logMesh("GridProjection", mesh, time);
    }

    void demo_Poisson()
    {
        ct::Surface sur;
        sur.setInputCloud(demo::makeSphereCloud(16, 16, 0.4f));
        ct::PolygonMesh::Ptr mesh;
        float time = 0.f;
        sur.Poisson(6, 2, 4.0f, 1.1f, 6, 6, 1.0f, false, false, true, mesh, time);
        logMesh("Poisson", mesh, time);
    }

    void demo_MarchingCubesRBF()
    {
        ct::Surface sur;
        sur.setInputCloud(demo::makeSphereCloud(12, 12, 0.3f));
        ct::PolygonMesh::Ptr mesh;
        float time = 0.f;
        sur.MarchingCubesRBF(0.0f, 12, 12, 12, 0.1f, 0.02f, mesh, time);
        logMesh("MarchingCubesRBF", mesh, time);
    }

    void demo_MarchingCubesHoppe()
    {
        ct::Surface sur;
        sur.setInputCloud(demo::makeSphereCloud(12, 12, 0.3f));
        ct::PolygonMesh::Ptr mesh;
        float time = 0.f;
        sur.MarchingCubesHoppe(0.0f, 12, 12, 12, 0.1f, 0.0f, mesh, time);
        logMesh("MarchingCubesHoppe", mesh, time);
    }

    void demo_ConvexHull()
    {
        ct::Surface sur;
        sur.setInputCloud(demo::makeSphereCloud(16, 16, 0.4f));
        ct::PolygonMesh::Ptr mesh;
        float time = 0.f;
        sur.ConvexHull(false, 3, mesh, time);
        logMesh("ConvexHull", mesh, time);
    }

    void demo_ConcaveHull()
    {
        ct::Surface sur;
        sur.setInputCloud(demo::makePlaneCloud(16, 0.04f));
        ct::PolygonMesh::Ptr mesh;
        float time = 0.f;
        sur.ConcaveHull(0.08, true, 2, mesh, time);
        logMesh("ConcaveHull", mesh, time);
    }

    void demo_EarClipping()
    {
        ct::Surface sur;
        sur.setInputCloud(demo::makeSphereCloud(12, 12, 0.3f));
        ct::PolygonMesh::Ptr hull;
        float t_hull = 0.f, t_ear = 0.f;
        sur.ConvexHull(false, 3, hull, t_hull);
        ct::PolygonMesh::Ptr mesh;
        sur.EarClipping(hull, mesh, t_ear);
        logMesh("EarClipping", mesh, t_ear);
    }
}  // namespace

int main()
{
    std::cout << "======== demo_surface (surface.h) ========\n";
    demo_GreedyProjectionTriangulation();
    demo_GridProjection();
    demo_Poisson();
    demo_MarchingCubesRBF();
    demo_MarchingCubesHoppe();
    demo_ConvexHull();
    demo_ConcaveHull();
    demo_EarClipping();
    std::cout << "done.\n";
    return 0;
}
