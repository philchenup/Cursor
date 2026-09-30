#include "ComputeTwoPointPoses.h"

#include <iostream>

int main()
{
    pcl::PointCloud<pcl::PointNormal> trajectory;
    trajectory.resize(2);

    trajectory[0].x = 0.f;
    trajectory[0].y = 0.f;
    trajectory[0].z = 1.f;
    trajectory[0].normal_x = 0.f;
    trajectory[0].normal_y = 0.f;
    trajectory[0].normal_z = 1.f;

    trajectory[1].x = 0.4f;
    trajectory[1].y = 0.1f;
    trajectory[1].z = 0.9f;
    trajectory[1].normal_x = 0.1f;
    trajectory[1].normal_y = 0.2f;
    trajectory[1].normal_z = 1.f;

    Eigen::Affine3f pose_start, pose_end;
    if (!computeTwoPointPoses(trajectory, pose_start, pose_end)) {
        std::cerr << "computeTwoPointPoses failed\n";
        return 1;
    }

    const Eigen::Vector3f x = pose_start.linear().col(0);
    std::cout << "X (start→end): " << x.transpose() << "\n"
              << "start t: " << pose_start.translation().transpose() << "\n"
              << "end   t: " << pose_end.translation().transpose() << "\n";
    return 0;
}
