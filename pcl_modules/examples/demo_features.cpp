/**
 * @file demo_features.cpp
 * @brief features.h 中每个公开接口的示例。
 */
#include "demo_utils.h"
#include "modules/features.h"

#include <iostream>

namespace
{
    ct::Features makeEstimator(const ct::Cloud::Ptr& cloud, int k, double radius)
    {
        ct::Features est;
        est.setInputCloud(cloud);
        est.setSearchSurface(cloud);
        est.setKSearch(k);
        est.setRadiusSearch(radius);
        return est;
    }

    void demo_boundingBoxAABB()
    {
        const ct::Box box = ct::Features::boundingBoxAABB(demo::makePlaneCloud());
        std::cout << "[ boundingBoxAABB ] " << box.width << " x " << box.height << " x " << box.depth << "\n";
    }

    void demo_boundingBoxOBB()
    {
        const ct::Box box = ct::Features::boundingBoxOBB(demo::makeSphereCloud());
        std::cout << "[ boundingBoxOBB ] " << box.width << " x " << box.height << " x " << box.depth << "\n";
    }

    void demo_boundingBoxAdjust()
    {
        Eigen::Affine3f t = Eigen::Affine3f::Identity();
        t.translation() << 0.1f, 0.0f, 0.0f;
        const ct::Box box = ct::Features::boundingBoxAdjust(demo::makePlaneCloud(), t);
        std::cout << "[ boundingBoxAdjust ] " << box.width << " x " << box.height << " x " << box.depth << "\n";
    }

    void demo_NormalEstimation()
    {
        ct::Features est = makeEstimator(demo::makePlaneCloud(20, 0.04f), 10, 0.0);
        ct::Cloud::Ptr out;
        float time = 0.f;
        est.NormalEstimation(0.f, 0.f, 10.f, out, time);
        demo::logCloud("NormalEstimation", out, time);
    }

    void demo_DifferenceOfNormalsEstimation()
    {
        ct::Cloud::Ptr cloud = demo::makeSphereCloud();
        ct::Features small = makeEstimator(cloud, 0, 0.08);
        ct::Features large = makeEstimator(cloud, 0, 0.16);
        ct::Cloud::Ptr n_small, n_large, don;
        float t1 = 0.f, t2 = 0.f, t3 = 0.f;
        small.NormalEstimation(0.f, 0.f, 1.f, n_small, t1);
        large.NormalEstimation(0.f, 0.f, 1.f, n_large, t2);
        ct::Features don_est = makeEstimator(cloud, 0, 0.12);
        don_est.DifferenceOfNormalsEstimation(n_small, n_large, don, t3);
        demo::logCloud("DifferenceOfNormalsEstimation", don, t3);
    }

    void demo_BoundaryEstimation()
    {
        ct::Features est = makeEstimator(demo::makePlaneCloud(20, 0.04f), 0, 0.08);
        ct::Cloud::Ptr out;
        float time = 0.f;
        est.BoundaryEstimation(80.f, out, time);
        demo::logCloud("BoundaryEstimation", out, time);
    }

    void demo_PFHEstimation()
    {
        ct::Features est = makeEstimator(demo::makeTinyFeatureCloud(), 8, 0.0);
        ct::FeatureType::Ptr feature;
        float time = 0.f;
        est.PFHEstimation(feature, time);
        std::cout << "[ PFHEstimation ] size=" << (feature && feature->pfh ? feature->pfh->size() : 0)
                  << " time=" << time << " ms\n";
    }

    void demo_FPFHEstimation()
    {
        ct::Features est = makeEstimator(demo::makeTinyFeatureCloud(), 8, 0.0);
        ct::FeatureType::Ptr feature;
        float time = 0.f;
        est.FPFHEstimation(feature, time);
        std::cout << "[ FPFHEstimation ] size=" << (feature && feature->fpfh ? feature->fpfh->size() : 0)
                  << " time=" << time << " ms\n";
    }

    void demo_VFHEstimation()
    {
        ct::Features est = makeEstimator(demo::makeTinyFeatureCloud(), 8, 0.0);
        ct::FeatureType::Ptr feature;
        float time = 0.f;
        est.VFHEstimation(Eigen::Vector3f(0.f, 0.f, 1.f), feature, time);
        std::cout << "[ VFHEstimation ] size=" << (feature && feature->vfh ? feature->vfh->size() : 0)
                  << " time=" << time << " ms\n";
    }

    void demo_ESFEstimation()
    {
        ct::Features est = makeEstimator(demo::makeTinyFeatureCloud(), 8, 0.0);
        ct::FeatureType::Ptr feature;
        float time = 0.f;
        est.ESFEstimation(feature, time);
        std::cout << "[ ESFEstimation ] size=" << (feature && feature->esf ? feature->esf->size() : 0)
                  << " time=" << time << " ms\n";
    }

    void demo_GASDEstimation()
    {
        ct::Features est = makeEstimator(demo::makeTinyFeatureCloud(), 8, 0.0);
        ct::FeatureType::Ptr feature;
        float time = 0.f;
        est.GASDEstimation(Eigen::Vector3f(0.f, 0.f, 1.f), 4, 8, 1, feature, time);
        std::cout << "[ GASDEstimation ] size=" << (feature && feature->gasd ? feature->gasd->size() : 0)
                  << " time=" << time << " ms\n";
    }

    void demo_GASDColorEstimation()
    {
        ct::Features est = makeEstimator(demo::makeTinyFeatureCloud(), 8, 0.0);
        ct::FeatureType::Ptr feature;
        float time = 0.f;
        est.GASDColorEstimation(Eigen::Vector3f(0.f, 0.f, 1.f), 4, 8, 1, 4, 8, 1, feature, time);
        std::cout << "[ GASDColorEstimation ] size=" << (feature && feature->gasdc ? feature->gasdc->size() : 0)
                  << " time=" << time << " ms\n";
    }

    void demo_RSDEstimation()
    {
        ct::Features est = makeEstimator(demo::makeTinyFeatureCloud(), 0, 0.15);
        ct::FeatureType::Ptr feature;
        float time = 0.f;
        est.RSDEstimation(5, 0.2, feature, time);
        std::cout << "[ RSDEstimation ] size=" << (feature && feature->rsd ? feature->rsd->size() : 0)
                  << " time=" << time << " ms\n";
    }

    void demo_GRSDEstimation()
    {
        ct::Features est = makeEstimator(demo::makeTinyFeatureCloud(), 0, 0.15);
        ct::FeatureType::Ptr feature;
        float time = 0.f;
        est.GRSDEstimation(feature, time);
        std::cout << "[ GRSDEstimation ] size=" << (feature && feature->grsd ? feature->grsd->size() : 0)
                  << " time=" << time << " ms\n";
    }

    void demo_CRHEstimation()
    {
        ct::Features est = makeEstimator(demo::makeTinyFeatureCloud(), 8, 0.0);
        ct::FeatureType::Ptr feature;
        float time = 0.f;
        est.CRHEstimation(Eigen::Vector3f(0.f, 0.f, 1.f), feature, time);
        std::cout << "[ CRHEstimation ] size=" << (feature && feature->crh ? feature->crh->size() : 0)
                  << " time=" << time << " ms\n";
    }

    void demo_CVFHEstimation()
    {
        ct::Features est = makeEstimator(demo::makeTinyFeatureCloud(), 8, 0.0);
        ct::FeatureType::Ptr feature;
        float time = 0.f;
        est.CVFHEstimation(Eigen::Vector3f(0.f, 0.f, 1.f), 0.08f, 0.05f, 0.15f, 0.1f, 10, true, feature, time);
        std::cout << "[ CVFHEstimation ] size=" << (feature && feature->vfh ? feature->vfh->size() : 0)
                  << " time=" << time << " ms\n";
    }

    void demo_ShapeContext3DEstimation()
    {
        ct::Features est = makeEstimator(demo::makeTinyFeatureCloud(), 0, 0.2);
        ct::FeatureType::Ptr feature;
        float time = 0.f;
        est.ShapeContext3DEstimation(0.02, 0.08, feature, time);
        std::cout << "[ ShapeContext3DEstimation ] size=" << (feature && feature->sc3d ? feature->sc3d->size() : 0)
                  << " time=" << time << " ms\n";
    }

    void demo_SHOTLocalReferenceFrameEstimation()
    {
        ct::Features est = makeEstimator(demo::makeTinyFeatureCloud(), 0, 0.15);
        ct::ReferenceFrame::Ptr lrf;
        float time = 0.f;
        est.SHOTLocalReferenceFrameEstimation(lrf, time);
        std::cout << "[ SHOTLocalReferenceFrameEstimation ] size=" << (lrf ? lrf->size() : 0)
                  << " time=" << time << " ms\n";
    }

    void demo_BOARDLocalReferenceFrameEstimation()
    {
        ct::Features est = makeEstimator(demo::makeTinyFeatureCloud(), 0, 0.15);
        ct::ReferenceFrame::Ptr lrf;
        float time = 0.f;
        est.BOARDLocalReferenceFrameEstimation(0.15f, true, 0.85f, 20, 0.2f, 0.1f, lrf, time);
        std::cout << "[ BOARDLocalReferenceFrameEstimation ] size=" << (lrf ? lrf->size() : 0)
                  << " time=" << time << " ms\n";
    }

    void demo_FLARELocalReferenceFrameEstimation()
    {
        ct::Features est = makeEstimator(demo::makeTinyFeatureCloud(), 0, 0.15);
        ct::ReferenceFrame::Ptr lrf;
        float time = 0.f;
        est.FLARELocalReferenceFrameEstimation(0.15f, 0.85f, 6, 6, lrf, time);
        std::cout << "[ FLARELocalReferenceFrameEstimation ] size=" << (lrf ? lrf->size() : 0)
                  << " time=" << time << " ms\n";
    }

    void demo_SHOTEstimation()
    {
        ct::Cloud::Ptr cloud = demo::makeTinyFeatureCloud();
        ct::Features est = makeEstimator(cloud, 0, 0.15);
        ct::ReferenceFrame::Ptr lrf;
        float t_lrf = 0.f, t_shot = 0.f;
        est.SHOTLocalReferenceFrameEstimation(lrf, t_lrf);
        ct::FeatureType::Ptr feature;
        est.SHOTEstimation(lrf, 0.15f, feature, t_shot);
        std::cout << "[ SHOTEstimation ] size=" << (feature && feature->shot ? feature->shot->size() : 0)
                  << " time=" << t_shot << " ms\n";
    }

    void demo_SHOTColorEstimation()
    {
        ct::Cloud::Ptr cloud = demo::makeTinyFeatureCloud();
        ct::Features est = makeEstimator(cloud, 0, 0.15);
        ct::ReferenceFrame::Ptr lrf;
        float t_lrf = 0.f, t_shot = 0.f;
        est.SHOTLocalReferenceFrameEstimation(lrf, t_lrf);
        ct::FeatureType::Ptr feature;
        est.SHOTColorEstimation(lrf, 0.15f, feature, t_shot);
        std::cout << "[ SHOTColorEstimation ] size=" << (feature && feature->shotc ? feature->shotc->size() : 0)
                  << " time=" << t_shot << " ms\n";
    }

    void demo_UniqueShapeContext()
    {
        ct::Cloud::Ptr cloud = demo::makeTinyFeatureCloud();
        ct::Features est = makeEstimator(cloud, 0, 0.2);
        ct::ReferenceFrame::Ptr lrf;
        float t_lrf = 0.f, t_usc = 0.f;
        est.SHOTLocalReferenceFrameEstimation(lrf, t_lrf);
        ct::FeatureType::Ptr feature;
        est.UniqueShapeContext(lrf, 0.02, 0.08, 0.15, feature, t_usc);
        std::cout << "[ UniqueShapeContext ] size=" << (feature && feature->usc ? feature->usc->size() : 0)
                  << " time=" << t_usc << " ms\n";
    }
}  // namespace

int main()
{
    std::cout.setf(std::ios::unitbuf);
    std::cout << "======== demo_features (features.h) ========\n";
    demo_boundingBoxAABB();
    demo_boundingBoxOBB();
    demo_boundingBoxAdjust();
    demo_PFHEstimation();
    demo_VFHEstimation();
    demo_ESFEstimation();
    demo_RSDEstimation();
    demo_CRHEstimation();
    demo_CVFHEstimation();
    demo_SHOTLocalReferenceFrameEstimation();
    demo_BOARDLocalReferenceFrameEstimation();
    demo_FLARELocalReferenceFrameEstimation();
    demo_BoundaryEstimation();
    demo_GASDEstimation();
    demo_GASDColorEstimation();
    demo_GRSDEstimation();
    demo_ShapeContext3DEstimation();
    demo_SHOTEstimation();
    demo_SHOTColorEstimation();
    demo_UniqueShapeContext();
    demo_FPFHEstimation();
    demo_NormalEstimation();
    demo_DifferenceOfNormalsEstimation();
    std::cout << "done.\n";
    return 0;
}
