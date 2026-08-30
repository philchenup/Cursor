#ifndef UNWRAP_JOINTS_NEAREST_H
#define UNWRAP_JOINTS_NEAREST_H

#include <cmath>
#include <cstddef>
#include <limits>

#include <Eigen/Core>

/**
 * @file UnwrapJointsNearest.h
 *
 * Robotics Library 的 `JacobianInverseKinematics::solve()` 在收敛后会调用
 * `kinematic->normalize(q)`。对转动关节，`Revolute::normalize` 做的是：
 *
 * @code
 * q = std::remainder(q, 2π);   // 结果落在 [-π, π]
 * if (q < min) q += 2π;
 * else if (q > max) q -= 2π;
 * @endcode
 *
 * 当限位是 ±360°（即 ±2π）时，[-π, π] 已经落在限位内，于是解被永久折回
 * ±180°。末端位姿相同，但相对真实关节角（例如 200°）会差一整圈，关节空间
 * 规划就会走出大幅度轨迹。
 *
 * 本函数在 IK 成功后把每个转动关节改写成 `q + 2πk`，在限位内选离 seed
 * 最近的那一圈。
 */

namespace unwrap_joints_detail {

constexpr double kTwoPi = 6.283185307179586476925286766559;
constexpr double kEps = 1e-12;

inline double UnwrapOneNearest(double q, double seed, double qMin, double qMax)
{
    if (qMax < qMin) {
        const double tmp = qMin;
        qMin = qMax;
        qMax = tmp;
    }

    const int kLo = static_cast<int>(std::floor((qMin - q) / kTwoPi)) - 1;
    const int kHi = static_cast<int>(std::ceil((qMax - q) / kTwoPi)) + 1;

    bool found = false;
    double best = q;
    double bestDist = std::numeric_limits<double>::infinity();
    int bestAbsK = std::numeric_limits<int>::max();

    for (int k = kLo; k <= kHi; ++k) {
        const double cand = q + static_cast<double>(k) * kTwoPi;
        if (cand < qMin - kEps || cand > qMax + kEps) {
            continue;
        }

        const double dist = std::fabs(cand - seed);
        const int absK = k >= 0 ? k : -k;
        if (!found || dist + kEps < bestDist ||
            (std::fabs(dist - bestDist) <= kEps && absK < bestAbsK)) {
            found = true;
            best = cand;
            bestDist = dist;
            bestAbsK = absK;
        }
    }

    return found ? best : q;
}

inline bool SameSize(Eigen::Index a, Eigen::Index b, Eigen::Index c, Eigen::Index d)
{
    return a == b && a == c && a == d && a >= 0;
}

} // namespace unwrap_joints_detail

/**
 * @brief 将关节角展开到离 seed 最近、且落在 [qMin, qMax] 内的 2π 周期副本。
 *
 * 只处理 `revolute[i] != 0` 的自由度；`revolute == nullptr` 时全部按转动关节处理。
 * 地轨等移动关节应传入 mask 并标成 false，否则大行程限位可能被误加 2π。
 *
 * @return 尺寸一致则改写 q 并返回 true；否则不改 q，返回 false。
 */
template <typename DerivedQ, typename DerivedSeed, typename DerivedMin, typename DerivedMax>
bool unwrapJointsNearestToSeed(Eigen::MatrixBase<DerivedQ>& q,
                               const Eigen::MatrixBase<DerivedSeed>& seed,
                               const Eigen::MatrixBase<DerivedMin>& qMin,
                               const Eigen::MatrixBase<DerivedMax>& qMax,
                               const bool* revolute)
{
    EIGEN_STATIC_ASSERT_VECTOR_ONLY(DerivedQ);
    EIGEN_STATIC_ASSERT_VECTOR_ONLY(DerivedSeed);
    EIGEN_STATIC_ASSERT_VECTOR_ONLY(DerivedMin);
    EIGEN_STATIC_ASSERT_VECTOR_ONLY(DerivedMax);

    const Eigen::Index n = q.size();
    if (!unwrap_joints_detail::SameSize(n, seed.size(), qMin.size(), qMax.size())) {
        return false;
    }

    for (Eigen::Index i = 0; i < n; ++i) {
        if (revolute != nullptr && !revolute[static_cast<std::size_t>(i)]) {
            continue;
        }
        q.derived()(i) = unwrap_joints_detail::UnwrapOneNearest(
            static_cast<double>(q.derived()(i)),
            static_cast<double>(seed.derived()(i)),
            static_cast<double>(qMin.derived()(i)),
            static_cast<double>(qMax.derived()(i)));
    }
    return true;
}

template <typename DerivedQ, typename DerivedSeed, typename DerivedMin, typename DerivedMax>
bool unwrapJointsNearestToSeed(Eigen::MatrixBase<DerivedQ>& q,
                               const Eigen::MatrixBase<DerivedSeed>& seed,
                               const Eigen::MatrixBase<DerivedMin>& qMin,
                               const Eigen::MatrixBase<DerivedMax>& qMax)
{
    return unwrapJointsNearestToSeed(q, seed, qMin, qMax, static_cast<const bool*>(nullptr));
}

/**
 * @brief 跳过前 `prismaticCount` 个自由度（地轨），其余按转动关节展开。
 */
template <typename DerivedQ, typename DerivedSeed, typename DerivedMin, typename DerivedMax>
bool unwrapJointsNearestToSeed(Eigen::MatrixBase<DerivedQ>& q,
                               const Eigen::MatrixBase<DerivedSeed>& seed,
                               const Eigen::MatrixBase<DerivedMin>& qMin,
                               const Eigen::MatrixBase<DerivedMax>& qMax,
                               int prismaticCount)
{
    const Eigen::Index n = q.size();
    if (!unwrap_joints_detail::SameSize(n, seed.size(), qMin.size(), qMax.size())) {
        return false;
    }
    if (prismaticCount < 0 || prismaticCount > static_cast<int>(n)) {
        return false;
    }

    for (Eigen::Index i = static_cast<Eigen::Index>(prismaticCount); i < n; ++i) {
        q.derived()(i) = unwrap_joints_detail::UnwrapOneNearest(
            static_cast<double>(q.derived()(i)),
            static_cast<double>(seed.derived()(i)),
            static_cast<double>(qMin.derived()(i)),
            static_cast<double>(qMax.derived()(i)));
    }
    return true;
}

#endif // UNWRAP_JOINTS_NEAREST_H
