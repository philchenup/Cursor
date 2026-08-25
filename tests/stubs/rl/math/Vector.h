#ifndef RL_MATH_VECTOR_STUB_H
#define RL_MATH_VECTOR_STUB_H

// Test-only stand-in for <rl/math/Vector.h> so the conversion can compile
// without installing the Robotics Library. Types match RL 0.7.

#include <Eigen/Core>

namespace rl
{
namespace math
{

typedef double Real;

typedef ::Eigen::Matrix<Real, ::Eigen::Dynamic, 1> Vector;
typedef ::Eigen::Matrix<Real, 2, 1> Vector2;
typedef ::Eigen::Matrix<Real, 3, 1> Vector3;
typedef ::Eigen::Matrix<Real, 4, 1> Vector4;
typedef ::Eigen::Matrix<Real, 6, 1> Vector6;

} // namespace math
} // namespace rl

#endif // RL_MATH_VECTOR_STUB_H
