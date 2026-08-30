#include "UnwrapJointsNearest.h"

#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {

int gFailures = 0;

constexpr double kDeg = 3.14159265358979323846 / 180.0;
constexpr double kTwoPi = 6.283185307179586476925286766559;

void expectTrue(bool cond, const char* msg)
{
    if (!cond) {
        std::cerr << "FAIL: " << msg << std::endl;
        ++gFailures;
    }
}

void expectNear(double actual, double expected, const char* msg, double tol = 1e-9)
{
    if (std::fabs(actual - expected) > tol) {
        std::cerr << "FAIL: " << msg << " " << actual << " != " << expected << std::endl;
        ++gFailures;
    }
}

// 复现 RL Revolute::normalize：remainder 到 [-π, π]，再按限位加减 2π。
double RlNormalizeRevolute(double q, double qMin, double qMax)
{
    q = std::remainder(q, kTwoPi);
    if (q < qMin) {
        q += kTwoPi;
    } else if (q > qMax) {
        q -= kTwoPi;
    }
    return q;
}

} // namespace

int main()
{
    const double lim = 360.0 * kDeg;

    // RL 把 200° 折成 -160°；展开后应回到 200°，规划步长约为 0 而不是 360°。
    {
        Eigen::VectorXd q(1), seed(1), qMin(1), qMax(1);
        seed << 200.0 * kDeg;
        q << RlNormalizeRevolute(200.0 * kDeg, -lim, lim);
        qMin << -lim;
        qMax << lim;
        expectNear(q(0), -160.0 * kDeg, "RL normalize 200deg -> -160deg", 1e-9);
        expectTrue(unwrapJointsNearestToSeed(q, seed, qMin, qMax), "unwrap 200deg");
        expectNear(q(0), 200.0 * kDeg, "unwrap back to 200deg");
        expectNear(std::fabs(q(0) - seed(0)), 0.0, "planning delta after unwrap");
    }

    // 种子在 -200°，IK 给出 +160°（同一姿态），应回到 -200°。
    {
        Eigen::VectorXd q(1), seed(1), qMin(1), qMax(1);
        seed << -200.0 * kDeg;
        q << RlNormalizeRevolute(-200.0 * kDeg, -lim, lim);
        qMin << -lim;
        qMax << lim;
        expectNear(q(0), 160.0 * kDeg, "RL normalize -200deg -> 160deg", 1e-9);
        expectTrue(unwrapJointsNearestToSeed(q, seed, qMin, qMax), "unwrap -200deg");
        expectNear(q(0), -200.0 * kDeg, "unwrap back to -200deg");
    }

    // 种子 350°，IK 归一化成 -10°；不展开会绕一整圈。
    {
        Eigen::VectorXd q(1), seed(1), qMin(1), qMax(1);
        seed << 350.0 * kDeg;
        q << RlNormalizeRevolute(350.0 * kDeg, -lim, lim);
        qMin << -lim;
        qMax << lim;
        expectNear(q(0), -10.0 * kDeg, "RL normalize 350deg -> -10deg", 1e-9);
        expectTrue(unwrapJointsNearestToSeed(q, seed, qMin, qMax), "unwrap 350deg");
        expectNear(q(0), 350.0 * kDeg, "unwrap back to 350deg");
    }

    // 限位 [-85, 265]：200° 的 remainder 是 -160°，低于下限，RL 会加回 2π。
    {
        const double jMin = -85.0 * kDeg;
        const double jMax = 265.0 * kDeg;
        Eigen::VectorXd q(1), seed(1), qMin(1), qMax(1);
        seed << 200.0 * kDeg;
        q << RlNormalizeRevolute(200.0 * kDeg, jMin, jMax);
        qMin << jMin;
        qMax << jMax;
        expectNear(q(0), 200.0 * kDeg, "asymmetric limit keeps 200deg");
        expectTrue(unwrapJointsNearestToSeed(q, seed, qMin, qMax), "unwrap asymmetric");
        expectNear(q(0), 200.0 * kDeg, "asymmetric stays 200deg");
    }

    // 已在最近圈：10° 保持 10°。
    {
        Eigen::VectorXd q(1), seed(1), qMin(1), qMax(1);
        seed << 10.0 * kDeg;
        q << 10.0 * kDeg;
        qMin << -lim;
        qMax << lim;
        expectTrue(unwrapJointsNearestToSeed(q, seed, qMin, qMax), "already nearest");
        expectNear(q(0), 10.0 * kDeg, "no change at 10deg");
    }

    // 6 轴：J1/J5/J6 可 ±360°，模拟 IK 全被折到 ±180°。
    {
        Eigen::VectorXd q(6), seed(6), qMin(6), qMax(6);
        seed << 190.0 * kDeg, 20.0 * kDeg, -10.0 * kDeg, 30.0 * kDeg, -190.0 * kDeg, 350.0 * kDeg;
        q = seed;
        qMin.setConstant(-lim);
        qMax.setConstant(lim);
        for (int i = 0; i < 6; ++i) {
            q(i) = RlNormalizeRevolute(q(i), -lim, lim);
        }
        expectNear(q(0), -170.0 * kDeg, "J1 wrapped", 1e-8);
        expectNear(q(4), 170.0 * kDeg, "J5 wrapped", 1e-8);
        expectNear(q(5), -10.0 * kDeg, "J6 wrapped", 1e-8);
        expectTrue(unwrapJointsNearestToSeed(q, seed, qMin, qMax), "unwrap 6dof");
        for (int i = 0; i < 6; ++i) {
            expectNear(q(i), seed(i), "6dof joint restored");
        }
    }

    // 地轨（第 0 轴，米）不得被加 2π。
    {
        Eigen::VectorXd q(2), seed(2), qMin(2), qMax(2);
        seed << 1.2, 200.0 * kDeg;
        q << 1.2, RlNormalizeRevolute(200.0 * kDeg, -lim, lim);
        qMin << -2.0, -lim;
        qMax << 2.0, lim;
        expectTrue(unwrapJointsNearestToSeed(q, seed, qMin, qMax, /*prismaticCount=*/1),
                   "skip rail");
        expectNear(q(0), 1.2, "rail unchanged");
        expectNear(q(1), 200.0 * kDeg, "arm unwrapped");
    }

    // 尺寸不一致：失败且不改 q。
    {
        Eigen::VectorXd q(2), seed(1), qMin(2), qMax(2);
        q << 1.0, 2.0;
        seed << 1.0;
        qMin.setZero();
        qMax.setOnes();
        expectTrue(!unwrapJointsNearestToSeed(q, seed, qMin, qMax), "size mismatch");
        expectNear(q(0), 1.0, "q unchanged on mismatch");
        expectNear(q(1), 2.0, "q unchanged on mismatch 1");
    }

    // mask：只展开第二轴。
    {
        Eigen::VectorXd q(2), seed(2), qMin(2), qMax(2);
        seed << 200.0 * kDeg, 200.0 * kDeg;
        q << -160.0 * kDeg, -160.0 * kDeg;
        qMin.setConstant(-lim);
        qMax.setConstant(lim);
        const bool revolute[2] = {false, true};
        expectTrue(unwrapJointsNearestToSeed(q, seed, qMin, qMax, revolute), "mask");
        expectNear(q(0), -160.0 * kDeg, "masked joint stays wrapped");
        expectNear(q(1), 200.0 * kDeg, "unmasked joint unwrapped");
    }

    if (gFailures != 0) {
        std::cerr << gFailures << " failure(s)" << std::endl;
        return EXIT_FAILURE;
    }
    std::cout << "UnwrapJointsNearest tests passed" << std::endl;
    return EXIT_SUCCESS;
}
