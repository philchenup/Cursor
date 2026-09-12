#include "WeldWeaveInterpolation.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>

namespace {

constexpr float kTestEps = 1e-5f;

Eigen::Affine3f makePose(const Eigen::Vector3f& translation,
                         const Eigen::Matrix3f& rotation = Eigen::Matrix3f::Identity())
{
    Eigen::Affine3f pose = Eigen::Affine3f::Identity();
    pose.linear() = rotation;
    pose.translation() = translation;
    return pose;
}

bool almostEqual(const Eigen::Vector3f& a, const Eigen::Vector3f& b, float eps = kTestEps)
{
    return (a - b).norm() <= eps;
}

bool rotationAlmostEqual(const Eigen::Affine3f& a, const Eigen::Affine3f& b, float eps = 1e-4f)
{
    return (a.linear() - b.linear()).norm() <= eps;
}

int fail(const std::string& msg)
{
    std::cerr << "FAIL: " << msg << std::endl;
    return 1;
}

}  // namespace

int main()
{
    const Eigen::Affine3f start = makePose(Eigen::Vector3f(0.0f, 0.0f, 0.0f));
    const Eigen::Affine3f end = makePose(Eigen::Vector3f(100.0f, 0.0f, 0.0f));
    const float amplitude = 5.0f;
    const float chordLength = 20.0f;  // 半周期 = 10

    // 1) 剩余长度不足半周期：整段直线。
    {
        const Eigen::Affine3f shortEnd = makePose(Eigen::Vector3f(8.0f, 0.0f, 0.0f));
        const auto pts = interpolateWeldWeave(
            start, shortEnd, WeavePattern::Sine, amplitude, chordLength, 1.0f);
        if (pts.size() != 2) {
            return fail("short seam should be start + end only");
        }
        if (!almostEqual(pts.front().translation(), start.translation()) ||
            !almostEqual(pts.back().translation(), shortEnd.translation())) {
            return fail("short seam endpoints");
        }
        if (!rotationAlmostEqual(pts.front(), start) || !rotationAlmostEqual(pts.back(), start)) {
            return fail("short seam orientation");
        }
    }

    // 2) 正弦：Identity 姿态下 Z=世界Z，焊缝沿 X，摆动应为 Y（Z × X）。
    {
        const auto pts = interpolateWeldWeave(
            start, end, WeavePattern::Sine, amplitude, chordLength, 0.5f);
        if (pts.size() < 4) {
            return fail("sine path too short");
        }
        if (!almostEqual(pts.front().translation(), start.translation()) ||
            !almostEqual(pts.back().translation(), end.translation())) {
            return fail("sine endpoints");
        }

        float maxAbsY = 0.0f;
        const Eigen::Vector3f z = start.linear().col(2);
        for (const auto& p : pts) {
            if (!rotationAlmostEqual(p, start)) {
                return fail("sine pose orientation changed");
            }
            maxAbsY = std::max(maxAbsY, std::abs(p.translation().y()));
            if (std::abs(p.translation().z()) > kTestEps) {
                return fail("sine left the plane perpendicular to Z");
            }
            const float s = p.translation().x();
            const Eigen::Vector3f offset = p.translation() - start.translation() -
                                           s * Eigen::Vector3f::UnitX();
            if (std::abs(offset.dot(z)) > 1e-4f) {
                return fail("sine weave not perpendicular to Z");
            }
        }
        if (std::abs(maxAbsY - amplitude) > 0.05f) {
            return fail("sine amplitude mismatch");
        }

        const Eigen::Affine3f end95 = makePose(Eigen::Vector3f(95.0f, 0.0f, 0.0f));
        const auto pts95 = interpolateWeldWeave(
            start, end95, WeavePattern::Sine, amplitude, chordLength, 0.5f);
        bool sawStraight = false;
        for (const auto& p : pts95) {
            if (p.translation().x() > 90.0f + 1e-4f) {
                sawStraight = true;
                if (std::abs(p.translation().y()) > 1e-4f) {
                    return fail("remaining segment must be on centerline");
                }
            }
        }
        if (!sawStraight) {
            return fail("expected straight tail after 90");
        }
        if (!almostEqual(pts95.back().translation(), end95.translation())) {
            return fail("sine 95 endpoint");
        }
    }

    // 3) 三角：四分之一周期处应到达 +amplitude（沿 +Y）。
    {
        const auto pts = interpolateWeldWeave(
            start, end, WeavePattern::Triangle, amplitude, chordLength, 0.25f);
        bool foundPeak = false;
        for (const auto& p : pts) {
            const Eigen::Vector3f t = p.translation();
            if (std::abs(t.x() - 5.0f) < 1e-4f && std::abs(t.y() - amplitude) < 1e-4f) {
                foundPeak = true;
                break;
            }
        }
        if (!foundPeak) {
            return fail("triangle peak at T/4 not found");
        }
        if (!almostEqual(pts.front().translation(), start.translation()) ||
            !almostEqual(pts.back().translation(), end.translation())) {
            return fail("triangle endpoints");
        }
    }

    // 4) 零长度焊缝。
    {
        const auto pts = interpolateWeldWeave(
            start, start, WeavePattern::Sine, amplitude, chordLength);
        if (pts.size() != 1 || !almostEqual(pts[0].translation(), start.translation())) {
            return fail("degenerate seam");
        }
    }

    // 5) 空间斜焊缝，姿态 Z 朝世界 Z：摆动应在 XY 平面内且垂直于焊缝。
    {
        const Eigen::Affine3f a = makePose(Eigen::Vector3f(1.0f, 2.0f, 3.0f));
        const Eigen::Affine3f b = makePose(Eigen::Vector3f(41.0f, 32.0f, 3.0f));
        const auto pts = interpolateWeldWeave(
            a, b, WeavePattern::Triangle, amplitude, chordLength, 0.5f);
        if (!almostEqual(pts.front().translation(), a.translation()) ||
            !almostEqual(pts.back().translation(), b.translation())) {
            return fail("3d endpoints");
        }
        const Eigen::Vector3f weld = (b.translation() - a.translation()).normalized();
        const Eigen::Vector3f z = a.linear().col(2).normalized();
        float maxLat = 0.0f;
        for (const auto& p : pts) {
            if (!rotationAlmostEqual(p, a)) {
                return fail("3d orientation changed");
            }
            const float s = (p.translation() - a.translation()).dot(weld);
            const Eigen::Vector3f lat = p.translation() - a.translation() - s * weld;
            maxLat = std::max(maxLat, lat.norm());
            if (lat.norm() > kTestEps && std::abs(lat.normalized().dot(z)) > 1e-4f) {
                return fail("3d weave not perpendicular to Z");
            }
        }
        if (std::abs(maxLat - amplitude) > 0.05f) {
            return fail("3d projected amplitude mismatch");
        }
    }

    // 6) 姿态绕 X 旋转 90°：Z 落到世界 +Y，焊缝沿 X，摆动应为 Z×X = -世界Z。
    {
        Eigen::Matrix3f R;
        R = Eigen::AngleAxisf(static_cast<float>(0.5 * 3.14159265358979323846),
                              Eigen::Vector3f::UnitX());
        const Eigen::Affine3f s = makePose(Eigen::Vector3f(0.0f, 0.0f, 0.0f), R);
        const Eigen::Affine3f e = makePose(Eigen::Vector3f(40.0f, 0.0f, 0.0f), R);
        const auto pts = interpolateWeldWeave(
            s, e, WeavePattern::Sine, amplitude, chordLength, 0.5f);

        float maxAbsZ = 0.0f;
        const Eigen::Vector3f z = s.linear().col(2).normalized();
        for (const auto& p : pts) {
            if (!rotationAlmostEqual(p, s)) {
                return fail("rotated orientation changed");
            }
            maxAbsZ = std::max(maxAbsZ, std::abs(p.translation().z()));
            if (std::abs(p.translation().y()) > 1e-4f) {
                return fail("rotated weave should be perpendicular to pose Z (world Y)");
            }
            const Eigen::Vector3f offset =
                p.translation() - Eigen::Vector3f(p.translation().x(), 0.0f, 0.0f);
            if (offset.norm() > kTestEps && std::abs(offset.normalized().dot(z)) > 1e-4f) {
                return fail("rotated weave not perpendicular to Z");
            }
        }
        if (std::abs(maxAbsZ - amplitude) > 0.05f) {
            return fail("rotated amplitude mismatch");
        }
    }

    std::cout << "OK" << std::endl;
    return 0;
}
