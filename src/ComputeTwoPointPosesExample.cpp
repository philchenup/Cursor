#include "ComputeTwoPointPoses.h"

#include <iostream>

int main()
{
    // 立墙竖缝（输入高→低）。Auto 判定为立焊：Y 从下到上，X 水平。
    pcl::PointCloud<pcl::PointNormal> trajectory;
    trajectory.resize(2);

    trajectory[0].x = 0.f;
    trajectory[0].y = 0.f;
    trajectory[0].z = 2.f;
    trajectory[0].normal_x = 0.f;
    trajectory[0].normal_y = 1.f;
    trajectory[0].normal_z = 0.f;

    trajectory[1].x = 0.f;
    trajectory[1].y = 0.f;
    trajectory[1].z = 0.f;
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
    std::cout << "start z: " << pose_start.translation().z()
              << "  end z: " << pose_end.translation().z() << "\n"
              << "X (should be horizontal): " << R.col(0).transpose()
              << "  X·up=" << R.col(0).dot(opt.world_up) << "\n"
              << "Y (bottom→top):           " << R.col(1).transpose()
              << "  Y·up=" << R.col(1).dot(opt.world_up) << "\n";
    return 0;
}
