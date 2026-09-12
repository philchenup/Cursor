#include "WeldWeaveInterpolation.h"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace {

constexpr double kEps = 1e-12;

double twoPi()
{
    return 2.0 * std::acos(-1.0);
}

/**
 * @brief 将摆动方向投影到垂直于焊缝的平面并单位化。
 *        若投影退化，则用焊缝方向与世界轴构造一个垂直向量。
 */
Eigen::Vector3d orthonormalLateral(const Eigen::Vector3d& weldDir,
                                   const Eigen::Vector3d& weaveDir)
{
    Eigen::Vector3d lateral = weaveDir - weaveDir.dot(weldDir) * weldDir;
    if (lateral.squaredNorm() > kEps) {
        return lateral.normalized();
    }

    Eigen::Vector3d fallback = weldDir.cross(Eigen::Vector3d::UnitZ());
    if (fallback.squaredNorm() <= kEps) {
        fallback = weldDir.cross(Eigen::Vector3d::UnitX());
    }
    return fallback.normalized();
}

/**
 * @brief 三角波，一个周期内：0 → +1 → 0 → -1 → 0。
 * @param phase 沿焊缝的归一化相位 s / chordLength，可为任意实数
 */
double triangleWave(double phase)
{
    phase -= std::floor(phase);
    if (phase < 0.25) {
        return 4.0 * phase;
    }
    if (phase < 0.75) {
        return 2.0 - 4.0 * phase;
    }
    return 4.0 * phase - 4.0;
}

double sineWave(double phase)
{
    return std::sin(twoPi() * phase);
}

double weaveOffset(WeavePattern pattern, double phase)
{
    return (pattern == WeavePattern::Triangle) ? triangleWave(phase)
                                               : sineWave(phase);
}

Eigen::Vector3d pointOnWeave(const Eigen::Vector3d& start,
                             const Eigen::Vector3d& weldDir,
                             const Eigen::Vector3d& lateral,
                             WeavePattern pattern,
                             double amplitude,
                             double chordLength,
                             double s)
{
    const double phase = s / chordLength;
    const double offset = amplitude * weaveOffset(pattern, phase);
    return start + s * weldDir + offset * lateral;
}

void appendSampled(std::vector<Eigen::Vector3d>& points,
                   const Eigen::Vector3d& p)
{
    if (points.empty() || (p - points.back()).squaredNorm() > kEps) {
        points.push_back(p);
    }
}

}  // namespace

std::vector<Eigen::Vector3d> interpolateWeldWeave(
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& end,
    const Eigen::Vector3d& weaveDir,
    WeavePattern pattern,
    double amplitude,
    double chordLength,
    double sampleStep)
{
    std::vector<Eigen::Vector3d> points;
    points.push_back(start);

    const Eigen::Vector3d seam = end - start;
    const double length = seam.norm();
    if (length <= kEps) {
        return points;
    }

    const Eigen::Vector3d weldDir = seam / length;

    // 弦长非法时无法形成周期，整段按直线到达终点。
    if (!(chordLength > kEps) || !std::isfinite(chordLength) ||
        !std::isfinite(amplitude)) {
        points.push_back(end);
        return points;
    }

    const double halfPeriod = 0.5 * chordLength;
    // 完整半周期个数；剩余不足半周期的部分直线到达终点。
    const int halfCycles =
        static_cast<int>(std::floor((length + kEps) / halfPeriod));
    const double weaveEndS =
        std::min(length, static_cast<double>(halfCycles) * halfPeriod);

    if (weaveEndS <= kEps) {
        points.push_back(end);
        return points;
    }

    const Eigen::Vector3d lateral = orthonormalLateral(weldDir, weaveDir);
    double step = sampleStep;
    if (!(step > kEps) || !std::isfinite(step)) {
        step = chordLength / 32.0;
    }
    step = std::min(step, weaveEndS);

    const int nSteps =
        std::max(1, static_cast<int>(std::ceil(weaveEndS / step - kEps)));
    for (int i = 1; i <= nSteps; ++i) {
        const double s = weaveEndS * static_cast<double>(i) /
                         static_cast<double>(nSteps);
        appendSampled(points,
                      pointOnWeave(start, weldDir, lateral, pattern, amplitude,
                                   chordLength, s));
    }

    // 剩余不足半个周期：沿焊缝中心线直线到终点。
    if ((end - points.back()).squaredNorm() > kEps) {
        const double remain = length - weaveEndS;
        if (remain > kEps) {
            const int nRemain =
                std::max(1, static_cast<int>(std::ceil(remain / step - kEps)));
            for (int i = 1; i <= nRemain; ++i) {
                const double s =
                    weaveEndS + remain * static_cast<double>(i) /
                                    static_cast<double>(nRemain);
                appendSampled(points, start + s * weldDir);
            }
        }
        appendSampled(points, end);
    }

    return points;
}
