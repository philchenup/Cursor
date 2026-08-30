#include "UnwrapJointsNearest.h"

#include <Eigen/Core>

// 用法：JacobianInverseKinematics 成功后立刻展开，再交给规划 / 下发。
//
// auto solveIk = [&](const Eigen::Affine3f& pose,
//                    const rl::math::Vector& qSeed,
//                    rl::math::Vector& qOut) -> bool
// {
//     rl::mdl::JacobianInverseKinematics ik(kinematic);
//     ik.setDuration(std::chrono::milliseconds(this->ikTimeoutMs));
//     rl::math::Transform T;
//     Eigen::Affine3f temp = pose;
//     temp.translation() *= 0.001;  // mm -> m
//     T.matrix() = temp.matrix().cast<rl::math::Real>();
//     ik.addGoal(T, 0);
//     kinematic->setPosition(qSeed);
//     if (!ik.solve()) {
//         return false;
//     }
//     qOut = kinematic->getPosition();
//     // RL 已把转动关节 remainder 到 ±180°；按 seed 展开回 ±360° 限位内最近圈。
//     unwrapJointsNearestToSeed(qOut, qSeed,
//                              kinematic->getMinimum(),
//                              kinematic->getMaximum());
//     return true;
// };

void UnfoldIkSolutionNearSeed(Eigen::VectorXd& qOut,
                              const Eigen::VectorXd& qSeed,
                              const Eigen::VectorXd& qMin,
                              const Eigen::VectorXd& qMax)
{
    unwrapJointsNearestToSeed(qOut, qSeed, qMin, qMax);
}

// 7 轴：Joint0 地轨不展开，J1–J6 展开
void UnfoldArmNearSeedSkipRail(Eigen::VectorXd& qOut,
                               const Eigen::VectorXd& qSeed,
                               const Eigen::VectorXd& qMin,
                               const Eigen::VectorXd& qMax)
{
    unwrapJointsNearestToSeed(qOut, qSeed, qMin, qMax, /*prismaticCount=*/1);
}
