#include "WeldWeaveInterpolation.h"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

namespace {

bool almostEqual(const Eigen::Vector3d& a, const Eigen::Vector3d& b, double eps = 1e-9)
{
    return (a - b).norm() <= eps;
}

int fail(const std::string& msg)
{
    std::cerr << "FAIL: " << msg << std::endl;
    return 1;
}

}  // namespace

int main()
{
    const Eigen::Vector3d start(0.0, 0.0, 0.0);
    const Eigen::Vector3d end(100.0, 0.0, 0.0);
    const Eigen::Vector3d weaveDir(0.0, 1.0, 0.0);
    const double amplitude = 5.0;
    const double chordLength = 20.0;  // 半周期 = 10

    // 1) 剩余长度不足半周期：整段直线。
    {
        const Eigen::Vector3d shortEnd(8.0, 0.0, 0.0);
        const auto pts = interpolateWeldWeave(
            start, shortEnd, weaveDir, WeavePattern::Sine, amplitude, chordLength, 1.0);
        if (pts.size() != 2) {
            return fail("short seam should be start + end only");
        }
        if (!almostEqual(pts.front(), start) || !almostEqual(pts.back(), shortEnd)) {
            return fail("short seam endpoints");
        }
    }

    // 2) 正弦：完整半周期后直线到终点，侧向峰值接近幅度。
    {
        const auto pts = interpolateWeldWeave(
            start, end, weaveDir, WeavePattern::Sine, amplitude, chordLength, 0.5);
        if (pts.size() < 4) {
            return fail("sine path too short");
        }
        if (!almostEqual(pts.front(), start) || !almostEqual(pts.back(), end)) {
            return fail("sine endpoints");
        }

        double maxAbsY = 0.0;
        for (const auto& p : pts) {
            maxAbsY = std::max(maxAbsY, std::abs(p.y()));
            if (std::abs(p.z()) > 1e-9) {
                return fail("sine left the weave plane");
            }
        }
        if (std::abs(maxAbsY - amplitude) > 0.05) {
            return fail("sine amplitude mismatch");
        }

        // 最后一个半周期结束于 x=100? 100/10=10，正好整数半周期，应落在终点。
        // 改用 length=95：9 个半周期到 x=90，其后直线。
        const Eigen::Vector3d end95(95.0, 0.0, 0.0);
        const auto pts95 = interpolateWeldWeave(
            start, end95, weaveDir, WeavePattern::Sine, amplitude, chordLength, 0.5);
        bool sawStraight = false;
        for (std::size_t i = 0; i < pts95.size(); ++i) {
            if (pts95[i].x() > 90.0 + 1e-6) {
                sawStraight = true;
                if (std::abs(pts95[i].y()) > 1e-8) {
                    return fail("remaining segment must be on centerline");
                }
            }
        }
        if (!sawStraight) {
            return fail("expected straight tail after 90");
        }
        if (!almostEqual(pts95.back(), end95)) {
            return fail("sine 95 endpoint");
        }
    }

    // 3) 三角：四分之一周期处应到达 +amplitude。
    {
        const auto pts = interpolateWeldWeave(
            start, end, weaveDir, WeavePattern::Triangle, amplitude, chordLength, 0.25);
        bool foundPeak = false;
        for (const auto& p : pts) {
            if (std::abs(p.x() - 5.0) < 1e-6 && std::abs(p.y() - amplitude) < 1e-6) {
                foundPeak = true;
                break;
            }
        }
        if (!foundPeak) {
            return fail("triangle peak at T/4 not found");
        }
        if (!almostEqual(pts.front(), start) || !almostEqual(pts.back(), end)) {
            return fail("triangle endpoints");
        }
    }

    // 4) 零长度焊缝。
    {
        const auto pts = interpolateWeldWeave(
            start, start, weaveDir, WeavePattern::Sine, amplitude, chordLength);
        if (pts.size() != 1 || !almostEqual(pts[0], start)) {
            return fail("degenerate seam");
        }
    }

    // 5) 空间斜焊缝：摆动方向不垂直焊缝时，应投影后仍保持侧向幅度。
    {
        const Eigen::Vector3d a(1.0, 2.0, 3.0);
        const Eigen::Vector3d b(1.0 + 40.0, 2.0 + 30.0, 3.0);  // 长度 50，半周期 10 → 5 个半周期
        const Eigen::Vector3d dir(0.0, 0.0, 1.0);
        const auto pts = interpolateWeldWeave(
            a, b, dir, WeavePattern::Triangle, amplitude, chordLength, 0.5);
        if (!almostEqual(pts.front(), a) || !almostEqual(pts.back(), b)) {
            return fail("3d endpoints");
        }
        const Eigen::Vector3d weld = (b - a).normalized();
        double maxLat = 0.0;
        for (const auto& p : pts) {
            const double s = (p - a).dot(weld);
            const Eigen::Vector3d lat = p - a - s * weld;
            maxLat = std::max(maxLat, lat.norm());
        }
        if (std::abs(maxLat - amplitude) > 0.05) {
            return fail("3d projected amplitude mismatch");
        }
    }

    std::cout << "OK" << std::endl;
    return 0;
}
