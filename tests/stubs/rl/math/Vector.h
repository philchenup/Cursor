#ifndef RL_MATH_VECTOR_STUB_H
#define RL_MATH_VECTOR_STUB_H

#include <vector>

namespace rl {
namespace math {

class Vector
{
public:
    Vector() = default;
    explicit Vector(int n, double v = 0.0) : data_(static_cast<std::size_t>(n), v) {}

    int size() const { return static_cast<int>(data_.size()); }
    double& operator[](int i) { return data_[static_cast<std::size_t>(i)]; }
    const double& operator[](int i) const { return data_[static_cast<std::size_t>(i)]; }

private:
    std::vector<double> data_;
};

} // namespace math
} // namespace rl

#endif
