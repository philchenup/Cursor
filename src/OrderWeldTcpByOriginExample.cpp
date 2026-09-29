#include "OrderWeldTcpByOrigin.h"

#include <iostream>

int main()
{
    Eigen::Affine3f tcp_weld_start = Eigen::Affine3f::Identity();
    Eigen::Affine3f tcp_weld_end = Eigen::Affine3f::Identity();
    tcp_weld_start.translation() = Eigen::Vector3f(80.f, 10.f, 5.f);
    tcp_weld_end.translation() = Eigen::Vector3f(10.f, 0.f, 0.f);
    // 故意让初始 X 指向反方向
    tcp_weld_start.linear().col(0) = Eigen::Vector3f(-1.f, 0.f, 0.f);
    tcp_weld_start.linear().col(1) = Eigen::Vector3f(0.f, -1.f, 0.f);
    tcp_weld_end.linear() = tcp_weld_start.linear();

    orderWeldTcpByOrigin(tcp_weld_start, tcp_weld_end);

    const Eigen::Vector3f expected_x =
        (tcp_weld_end.translation() - tcp_weld_start.translation()).normalized();
    std::cout << "start " << tcp_weld_start.translation().transpose()
              << "  ||p||=" << tcp_weld_start.translation().norm() << "\n";
    std::cout << "end   " << tcp_weld_end.translation().transpose()
              << "  ||p||=" << tcp_weld_end.translation().norm() << "\n";
    std::cout << "X     " << tcp_weld_start.linear().col(0).transpose() << "\n";
    std::cout << "X==end-start " << std::boolalpha
              << (tcp_weld_start.linear().col(0).dot(expected_x) > 0.999f)
              << "\n";
    return 0;
}
