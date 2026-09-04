#include "filterreg.h"

#include <iostream>

int main()
{
    Eigen::MatrixX3f source(8, 3);
    Eigen::MatrixX3f target(8, 3);
    for (int i = 0; i < 8; ++i)
    {
        source(i, 0) = static_cast<float>(i);
        source(i, 1) = 0.1f * static_cast<float>(i);
        source(i, 2) = 0.f;
        target(i, 0) = source(i, 0) + 0.5f;
        target(i, 1) = source(i, 1) - 0.2f;
        target(i, 2) = 0.1f;
    }

    filterreg::Options opt;
    opt.max_iter = 20;
    opt.update_sigma2 = true;

    const filterreg::Result result = filterreg::registration(source, target, {}, opt);

    std::cout << "filterreg converged=" << result.converged
              << " iterations=" << result.iterations
              << " sigma2=" << result.sigma2 << "\n"
              << result.transformation.rot << "\n"
              << result.transformation.t.transpose() << "\n";
    return 0;
}
