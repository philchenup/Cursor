#include "ComputeTwoPointPoses.h"

#include <iostream>

int main()
{
    // 立焊：输入上→下，应改成自下而上，不内倾。
    Eigen::Affine3f pose_start = Eigen::Affine3f::Identity();
    pose_start.translation() = Eigen::Vector3f(0.f, 0.f, 2.f);
    pose_start.linear().col(2) = Eigen::Vector3f(0.f, 1.f, 0.f);

    Eigen::Affine3f pose_end = Eigen::Affine3f::Identity();
    pose_end.translation() = Eigen::Vector3f(0.f, 0.f, 0.f);
    pose_end.linear().col(2) = Eigen::Vector3f(0.f, 1.f, 0.f);

    ComputeTwoPointPosesOptions opt;
    opt.travel_angle_deg = 0.f;
    opt.inward_deg = 30.f;
    if (!computeTwoPointPoses(pose_start, pose_end, opt))
        return 1;

    // 平焊角缝：Y 朝上决定顺序，起终点向内倾 30°。
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

    std::cout << pose_start.matrix() << "\n\n" << pose_end.matrix() << "\n\n"
              << tcp_weld_start.matrix() << "\n\n" << tcp_weld_end.matrix()
              << "\n";
    return 0;
}
