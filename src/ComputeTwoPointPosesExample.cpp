#include "ComputeTwoPointPoses.h"

#include <iostream>

int main()
{
    // 立墙竖缝，输入从低到高。免示教会改成高→低下坡，枪头（+X）朝下。
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

    const Eigen::Vector3f head = pose_start.linear() * opt.tool_head_axis;
    std::cout << "start z: " << pose_start.translation().z()
              << "  end z: " << pose_end.translation().z() << "\n"
              << "X: " << pose_start.linear().col(0).transpose() << "\n"
              << "head·up: " << head.dot(opt.world_up) << "  (should be < 0)\n";
    return 0;
}
