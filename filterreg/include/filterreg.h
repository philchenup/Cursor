#pragma once
// Rigid FilterReg, ported from neka-nat/probreg (filterreg.py).
// Gao & Tedrake, "FilterReg: Robust and Efficient Probabilistic Point-Set
// Registration using Gaussian Filter and Twist Parameterization", CVPR 2019.
#pragma once

#include <Eigen/Core>

namespace filterreg {

    enum class Objective { PointToPoint, PointToPlane };

    struct RigidTransform {
        Eigen::Matrix3f rot = Eigen::Matrix3f::Identity();
        Eigen::Vector3f t = Eigen::Vector3f::Zero();

        Eigen::MatrixX3f apply(const Eigen::MatrixX3f& points) const;
    };

    struct Options {
        float sigma2 = -1.f;  // < 0: initialize from squared kernel sum
        bool update_sigma2 = false;
        float w = 0.f;  // uniform outlier weight, 0 <= w < 1
        Objective objective = Objective::PointToPoint;
        int max_iter = 50;
        float tol = 1e-3f;
        float min_sigma2 = 1e-4f;
        float alpha = 0.015f;  // disable blur if lattice is larger than n * alpha
        RigidTransform init;
    };

    struct Result {
        RigidTransform transformation;
        float sigma2 = 0.f;
        float q = 0.f;
        int iterations = 0;
        bool converged = false;
    };

    // source / target: N x 3, M x 3. target_normals may be empty unless pt2pl.
    Result registration(const Eigen::MatrixX3f& source,
        const Eigen::MatrixX3f& target,
        const Eigen::MatrixX3f& target_normals = Eigen::MatrixX3f(),
        const Options& opt = Options());

}  // namespace filterreg
