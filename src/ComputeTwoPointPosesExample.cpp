#include "ComputeTwoPointPoses.h"

#include <iostream>

int main()
{
    // 立墙竖缝：法向沿 +Y，行走沿 +Z。枪轴会略向 -Z 倾斜，X 保持向上。
    pcl::PointCloud<pcl::PointNormal> trajectory;
    trajectory.resize(2);

    trajectory[0].x = 0.f;
    trajectory[0].y = 0.f;
    trajectory[0].z = 0.f;
    trajectory[0].normal_x = 0.f;
    trajectory[0].normal_y = 1.f;
    trajectory[0].normal_z = 0.f;

    trajectory[1].x = 0.f;
    trajectory[1].y = 0.f;
    trajectory[1].z = 2.f;
    trajectory[1].normal_x = 0.f;
    trajectory[1].normal_y = 1.f;
    trajectory[1].normal_z = 0.f;

    ComputeTwoPointPosesOptions opt;
    Eigen::Affine3f pose_start, pose_end;
    if (!computeTwoPointPoses(trajectory, pose_start, pose_end, opt)) {
        std::cerr << "computeTwoPointPoses failed\n";
        return 1;
    }

    const Eigen::Matrix3f R = pose_start.linear();
    std::cout << "X (travel in torch plane): " << R.col(0).transpose() << "\n"
              << "Z (torch):                 " << R.col(2).transpose() << "\n"
              << "X·Z (should be ~0):        " << R.col(0).dot(R.col(2)) << "\n";
    return 0;
}
