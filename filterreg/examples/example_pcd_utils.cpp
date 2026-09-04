#include "pcd_utils.h"

#include <iostream>

int main()
{
    ct::Cloud::Ptr src(new ct::Cloud);
    ct::Cloud::Ptr tgt(new ct::Cloud);
    for (int i = 0; i < 40; ++i)
    {
        pcl::PointXYZRGBNormal p;
        p.x = static_cast<float>(i) * 0.01f;
        p.y = 0.002f * static_cast<float>(i);
        p.z = 0.1f;
        p.r = 255;
        p.g = 0;
        p.b = 0;
        src->push_back(p);
        p.x += 0.03f;
        p.y -= 0.01f;
        tgt->push_back(p);
    }
    src->width = static_cast<uint32_t>(src->size());
    src->height = 1;
    tgt->width = static_cast<uint32_t>(tgt->size());
    tgt->height = 1;

    ct::Cloud::Ptr down(new ct::Cloud);
    PcdUtils::pcd_voxel_down(src, down, 0.02);

    regPara rp;
    rp.initRegIter = 15;
    rp.maxCorrDis = 0.2;
    rp.refineIterations = 10;
    Eigen::Affine3f T = Eigen::Affine3f::Identity();
    PcdUtils::FilterReg(src, tgt, rp, T);

    std::cout << "pcd_utils downsampled=" << down->size()
              << " FilterReg T=\n" << T.matrix() << "\n";
    return 0;
}
