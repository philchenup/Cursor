#include "ComputeTwoPointPoses.h"

#include <iostream>
#include <utility>
#include <vector>

int main()
{
    // 模拟 SeamExtra::trajectoryCloud：每条焊缝一对 TCP。
    Eigen::Affine3f tcp_weld_start = Eigen::Affine3f::Identity();
    tcp_weld_start.translation() = Eigen::Vector3f(200.f, 0.f, 0.f);
    tcp_weld_start.linear().col(2) =
        Eigen::Vector3f(0.f, -0.70710678f, -0.70710678f);
    Eigen::Affine3f tcp_weld_end = Eigen::Affine3f::Identity();
    tcp_weld_end.translation() = Eigen::Vector3f(50.f, 0.f, 0.f);
    tcp_weld_end.linear().col(2) =
        Eigen::Vector3f(0.f, -0.70710678f, -0.70710678f);
    if (!computeWeldTcpStartEnd(tcp_weld_start, tcp_weld_end))
        return 1;

    std::vector<std::pair<Eigen::Affine3f, Eigen::Affine3f>> trajectory_cloud;
    trajectory_cloud.emplace_back(tcp_weld_start, tcp_weld_end);

    Eigen::Affine3f pose_start = Eigen::Affine3f::Identity();
    pose_start.translation() = Eigen::Vector3f(0.f, 0.f, 2.f);
    pose_start.linear().col(2) = Eigen::Vector3f(0.f, 1.f, 0.f);
    Eigen::Affine3f pose_end = Eigen::Affine3f::Identity();
    pose_end.translation() = Eigen::Vector3f(0.f, 0.f, 0.f);
    pose_end.linear().col(2) = Eigen::Vector3f(0.f, 1.f, 0.f);
    ComputeTwoPointPosesOptions opt;
    opt.travel_angle_deg = 0.f;
    if (!computeTwoPointPoses(pose_start, pose_end, opt))
        return 1;
    trajectory_cloud.emplace_back(pose_start, pose_end);

    for (const auto& se : trajectory_cloud)
        std::cout << se.first.matrix() << "\n\n" << se.second.matrix() << "\n\n";
    return 0;
}
