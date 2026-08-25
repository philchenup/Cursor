#ifndef RL_VECTOR_TO_STD_VECTOR_H
#define RL_VECTOR_TO_STD_VECTOR_H

#include <rl/math/Vector.h>

#include <cstddef>
#include <vector>

/**
 * @brief 将 Robotics Library 的列向量转换为 `std::vector<float>`。
 *
 * `rl::math::Vector` 是 `Eigen::Matrix<rl::math::Real, Dynamic, 1>`
 *（默认 `Real` 为 `double`）。每个元素按 `static_cast<float>` 截断后写入结果。
 * `Vector2` / `Vector3` / `Vector4` / `Vector6` 以及其它 Eigen 列/行向量同样适用。
 *
 * @param v 输入向量（允许空向量）
 * @return 与 `v.size()` 等长的 `std::vector<float>`
 *
 * @code
 * rl::math::Vector q(6);
 * q << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6;
 * std::vector<float> joints = rlVectorToStdVector(q);
 * @endcode
 */
template <typename Derived>
inline std::vector<float> rlVectorToStdVector(const Eigen::MatrixBase<Derived>& v)
{
    EIGEN_STATIC_ASSERT_VECTOR_ONLY(Derived);

    const Eigen::Index n = v.size();
    std::vector<float> out(static_cast<std::size_t>(n));
    if (n > 0) {
        Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, 1> >(out.data(), n) =
            v.derived().template cast<float>();
    }
    return out;
}

#endif // RL_VECTOR_TO_STD_VECTOR_H
