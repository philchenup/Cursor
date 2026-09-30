#ifndef GET_FLANGE_POSE_MM_ABC_H
#define GET_FLANGE_POSE_MM_ABC_H

#include <iostream>
#include <rl/math/Unit.h>
#include <rl/mdl/Kinematic.h>

/** 法兰位姿：mm + ABC 度。与 OperationalModel 相同：R = Rz(C)*Ry(B)*Rx(A)。 */
inline void printFlangePoseMmAbc(rl::mdl::Kinematic* mdl, std::size_t ee = 0)
{
    mdl->forwardPosition();
    const auto& T = mdl->getOperationalPosition(ee);
    const auto p = T.translation() * 1000.0;
    const auto abc = T.rotation().eulerAngles(2, 1, 0).reverse() * rl::math::RAD2DEG;
    std::cout << p.x() << " " << p.y() << " " << p.z() << " "
              << abc.x() << " " << abc.y() << " " << abc.z() << "\n";
}

#endif
