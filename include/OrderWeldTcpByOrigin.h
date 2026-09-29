#ifndef ORDER_WELD_TCP_BY_ORIGIN_H
#define ORDER_WELD_TCP_BY_ORIGIN_H

#include <utility>
#include <Eigen/Geometry>

/**
 * 按到原点距离重排焊接 TCP：近原点为起点，远原点为终点。
 * 随后把两端姿态的 X 设为起点→终点，Y = Z × X，保持右手系。
 */
inline void orderWeldTcpByOrigin(Eigen::Affine3f& tcp_weld_start,
                                 Eigen::Affine3f& tcp_weld_end)
{
    if (tcp_weld_start.translation().squaredNorm() >
        tcp_weld_end.translation().squaredNorm())
        std::swap(tcp_weld_start, tcp_weld_end);

    const Eigen::Vector3f x =
        (tcp_weld_end.translation() - tcp_weld_start.translation()).normalized();

    auto setX = [&](Eigen::Affine3f& tcp) {
        Eigen::Matrix3f& R = tcp.linear();
        Eigen::Vector3f y = R.col(2).cross(x);
        if (y.squaredNorm() < 1e-12f)
            y = R.col(1);
        else
            y.normalize();
        R.col(0) = x;
        R.col(1) = y;
        R.col(2) = x.cross(y).normalized();
    };
    setX(tcp_weld_start);
    setX(tcp_weld_end);
}

#endif // ORDER_WELD_TCP_BY_ORIGIN_H
