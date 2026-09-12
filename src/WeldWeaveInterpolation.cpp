#include "WeldWeaveInterpolation.h"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace {

constexpr float kEps = 1e-6f;

float twoPi()
{
    return 2.0f * std::acos(-1.0f);
}

Eigen::Vector3f poseZAxis(const Eigen::Affine3f& pose)
{
    const Eigen::Vector3f z = pose.linear().col(2);
    if (z.squaredNorm() > kEps) {
        return z.normalized();
    }
    return Eigen::Vector3f::UnitZ();
}

/**
 * @brief 摆动方向：垂直于姿态 Z 轴，且垂直于焊缝前进方向。
 *        优先 Z × weldDir；Z 与焊缝平行时用姿态 Y（已垂直于 Z）。
 */
Eigen::Vector3f weaveDirPerpendicularToZ(const Eigen::Affine3f& pose,
                                         const Eigen::Vector3f& weldDir)
{
    const Eigen::Vector3f z = poseZAxis(pose);
    Eigen::Vector3f lateral = z.cross(weldDir);
    if (lateral.squaredNorm() > kEps) {
        return lateral.normalized();
    }

    auto projectPerpToWeld = [&weldDir](Eigen::Vector3f v) {
        v -= v.dot(weldDir) * weldDir;
        return v;
    };

    lateral = projectPerpToWeld(pose.linear().col(1));
    if (lateral.squaredNorm() > kEps) {
        return lateral.normalized();
    }

    lateral = projectPerpToWeld(pose.linear().col(0));
    if (lateral.squaredNorm() > kEps) {
        return lateral.normalized();
    }

    lateral = weldDir.cross(Eigen::Vector3f::UnitZ());
    if (lateral.squaredNorm() <= kEps) {
        lateral = weldDir.cross(Eigen::Vector3f::UnitX());
    }
    return lateral.normalized();
}

/**
 * @brief 三角波，一个周期内：0 → +1 → 0 → -1 → 0。
 * @param phase 沿焊缝的归一化相位 s / chordLength，可为任意实数
 */
float triangleWave(float phase)
{
    phase -= std::floor(phase);
    if (phase < 0.25f) {
        return 4.0f * phase;
    }
    if (phase < 0.75f) {
        return 2.0f - 4.0f * phase;
    }
    return 4.0f * phase - 4.0f;
}

float sineWave(float phase)
{
    return std::sin(twoPi() * phase);
}

float weaveOffset(WeavePattern pattern, float phase)
{
    return (pattern == WeavePattern::Triangle) ? triangleWave(phase)
                                               : sineWave(phase);
}

Eigen::Vector3f pointOnWeave(const Eigen::Vector3f& origin,
                             const Eigen::Vector3f& weldDir,
                             const Eigen::Vector3f& lateral,
                             WeavePattern pattern,
                             float amplitude,
                             float chordLength,
                             float s)
{
    const float phase = s / chordLength;
    const float offset = amplitude * weaveOffset(pattern, phase);
    return origin + s * weldDir + offset * lateral;
}

Eigen::Affine3f poseWithTranslation(const Eigen::Affine3f& orientationSource,
                                    const Eigen::Vector3f& translation)
{
    Eigen::Affine3f pose = orientationSource;
    pose.translation() = translation;
    return pose;
}

void appendSampled(std::vector<Eigen::Affine3f>& poses, const Eigen::Affine3f& pose)
{
    if (poses.empty() ||
        (pose.translation() - poses.back().translation()).squaredNorm() > kEps) {
        poses.push_back(pose);
    }
}

}  // namespace

std::vector<Eigen::Affine3f> interpolateWeldWeave(
    const Eigen::Affine3f& start,
    const Eigen::Affine3f& end,
    WeavePattern pattern,
    float amplitude,
    float chordLength,
    float sampleStep)
{
    std::vector<Eigen::Affine3f> poses;
    poses.push_back(start);

    const Eigen::Vector3f origin = start.translation();
    const Eigen::Vector3f seam = end.translation() - origin;
    const float length = seam.norm();
    if (length <= kEps) {
        return poses;
    }

    const Eigen::Vector3f weldDir = seam / length;

    // 弦长非法时无法形成周期，整段按直线到达终点。
    if (!(chordLength > kEps) || !std::isfinite(chordLength) ||
        !std::isfinite(amplitude)) {
        appendSampled(poses, end);
        return poses;
    }

    const float halfPeriod = 0.5f * chordLength;
    // 完整半周期个数；剩余不足半周期的部分直线到达终点。
    const int halfCycles =
        static_cast<int>(std::floor((length + kEps) / halfPeriod));
    const float weaveEndS =
        std::min(length, static_cast<float>(halfCycles) * halfPeriod);

    if (weaveEndS <= kEps) {
        appendSampled(poses, end);
        return poses;
    }

    const Eigen::Vector3f lateral = weaveDirPerpendicularToZ(start, weldDir);
    float step = sampleStep;
    if (!(step > kEps) || !std::isfinite(step)) {
        step = chordLength / 32.0f;
    }
    step = std::min(step, weaveEndS);

    const int nSteps =
        std::max(1, static_cast<int>(std::ceil(weaveEndS / step - kEps)));
    for (int i = 1; i <= nSteps; ++i) {
        const float s = weaveEndS * static_cast<float>(i) /
                        static_cast<float>(nSteps);
        appendSampled(poses,
                      poseWithTranslation(start,
                                          pointOnWeave(origin, weldDir, lateral,
                                                       pattern, amplitude,
                                                       chordLength, s)));
    }

    // 剩余不足半个周期：沿焊缝中心线直线到终点，方向保持不变。
    if ((end.translation() - poses.back().translation()).squaredNorm() > kEps) {
        const float remain = length - weaveEndS;
        if (remain > kEps) {
            const int nRemain =
                std::max(1, static_cast<int>(std::ceil(remain / step - kEps)));
            for (int i = 1; i < nRemain; ++i) {
                const float s =
                    weaveEndS + remain * static_cast<float>(i) /
                                    static_cast<float>(nRemain);
                appendSampled(poses, poseWithTranslation(start, origin + s * weldDir));
            }
        }
        appendSampled(poses, end);
    }

    return poses;
}
