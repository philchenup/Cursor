#include "RlVectorToStdVector.h"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <vector>

namespace {

int gFailures = 0;

void expectSize(const std::vector<float>& actual, std::size_t expected, const char* msg)
{
    if (actual.size() != expected) {
        std::cerr << "FAIL: " << msg << " size " << actual.size()
                  << " != " << expected << std::endl;
        ++gFailures;
    }
}

void expectNear(float actual, float expected, const char* msg)
{
    const float tol = 1e-6f;
    if (std::isnan(expected)) {
        if (!std::isnan(actual)) {
            std::cerr << "FAIL: " << msg << " expected NaN, got " << actual << std::endl;
            ++gFailures;
        }
        return;
    }
    if (std::isinf(expected)) {
        if (!(std::isinf(actual) && ((actual > 0) == (expected > 0)))) {
            std::cerr << "FAIL: " << msg << " expected inf, got " << actual << std::endl;
            ++gFailures;
        }
        return;
    }
    if (std::fabs(actual - expected) > tol) {
        std::cerr << "FAIL: " << msg << " " << actual << " != " << expected << std::endl;
        ++gFailures;
    }
}

} // namespace

int main()
{
    {
        rl::math::Vector empty;
        const std::vector<float> out = rlVectorToStdVector(empty);
        expectSize(out, 0, "empty Vector");
    }

    {
        rl::math::Vector q(6);
        q << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6;
        const std::vector<float> out = rlVectorToStdVector(q);
        expectSize(out, 6, "Vector(6)");
        expectNear(out[0], 0.1f, "q[0]");
        expectNear(out[1], 0.2f, "q[1]");
        expectNear(out[2], 0.3f, "q[2]");
        expectNear(out[3], 0.4f, "q[3]");
        expectNear(out[4], 0.5f, "q[4]");
        expectNear(out[5], 0.6f, "q[5]");
    }

    {
        rl::math::Vector3 p;
        p << 1.25, -2.5, 3.75;
        const std::vector<float> out = rlVectorToStdVector(p);
        expectSize(out, 3, "Vector3");
        expectNear(out[0], 1.25f, "p.x");
        expectNear(out[1], -2.5f, "p.y");
        expectNear(out[2], 3.75f, "p.z");
    }

    {
        rl::math::Vector6 twist;
        twist << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0;
        const std::vector<float> out = rlVectorToStdVector(twist);
        expectSize(out, 6, "Vector6");
        for (int i = 0; i < 6; ++i) {
            expectNear(out[static_cast<std::size_t>(i)], static_cast<float>(i + 1), "twist");
        }
    }

    {
        rl::math::Vector v(3);
        v << std::numeric_limits<double>::quiet_NaN(),
            std::numeric_limits<double>::infinity(),
            -std::numeric_limits<double>::infinity();
        const std::vector<float> out = rlVectorToStdVector(v);
        expectSize(out, 3, "specials");
        expectNear(out[0], std::numeric_limits<float>::quiet_NaN(), "nan");
        expectNear(out[1], std::numeric_limits<float>::infinity(), "+inf");
        expectNear(out[2], -std::numeric_limits<float>::infinity(), "-inf");
    }

    {
        // Double values that need a narrowing cast still round-trip as float.
        rl::math::Vector v(1);
        v[0] = 1.0 / 3.0;
        const std::vector<float> out = rlVectorToStdVector(v);
        expectSize(out, 1, "narrow");
        expectNear(out[0], static_cast<float>(1.0 / 3.0), "1/3");
    }

    if (gFailures == 0) {
        std::cout << "RlVectorToStdVector: all tests passed" << std::endl;
        return EXIT_SUCCESS;
    }

    std::cerr << gFailures << " failure(s)" << std::endl;
    return EXIT_FAILURE;
}
