#ifndef JAKA_RL_COMPAT_H
#define JAKA_RL_COMPAT_H

/**
 * Robotics Library 的 rl::math::Vector 兼容层。
 * 若工程已安装 RL（或 include 路径中存在 rl/math/Vector.h），则使用官方类型；
 * 否则提供一个与 Eigen/RL 接口兼容的最小 Vector，便于独立编译与测试。
 */
#if defined(JAKA_HAS_RL) || __has_include(<rl/math/Vector.h>)
#include <rl/math/Vector.h>
#else
#include <cstddef>
#include <vector>

namespace rl {
namespace math {

class Vector
{
public:
    Vector() = default;
    explicit Vector(int n, double value = 0.0)
        : data_(static_cast<std::size_t>(n), value)
    {
    }

    int size() const { return static_cast<int>(data_.size()); }

    double& operator[](int i) { return data_[static_cast<std::size_t>(i)]; }
    const double& operator[](int i) const { return data_[static_cast<std::size_t>(i)]; }

    double& operator()(int i) { return (*this)[i]; }
    const double& operator()(int i) const { return (*this)[i]; }

private:
    std::vector<double> data_;
};

} // namespace math
} // namespace rl
#endif

#endif // JAKA_RL_COMPAT_H
