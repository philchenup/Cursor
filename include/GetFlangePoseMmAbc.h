#ifndef GET_FLANGE_POSE_MM_ABC_H
#define GET_FLANGE_POSE_MM_ABC_H

#include <rl/math/Transform.h>
#include <rl/math/Unit.h>
#include <rl/mdl/Kinematic.h>

/**
 * 末端法兰位姿：毫米 + 欧拉角 ABC（度）。
 *
 * 与 MainWindow / OperationalModel::data 同一套约定：
 *   T = mdl->getOperationalPosition(ee)   // Robotics Library，平移为米
 *   ABC = rotation().eulerAngles(2, 1, 0).reverse() * RAD2DEG
 *         即 R = Rz(C) * Ry(B) * Rx(A)
 *
 * 调用前若刚改过关节，先 mdl->setPosition(q) 再 forwardPosition()。
 */
struct FlangePoseMmAbc {
    double x_mm = 0.0;
    double y_mm = 0.0;
    double z_mm = 0.0;
    double a_deg = 0.0;
    double b_deg = 0.0;
    double c_deg = 0.0;
};

inline bool getFlangePoseMmAbc(rl::mdl::Kinematic* mdl,
                               FlangePoseMmAbc& out,
                               std::size_t ee = 0)
{
    if (!mdl || ee >= mdl->getOperationalDof())
        return false;

    mdl->forwardPosition();
    const rl::math::Transform& T = mdl->getOperationalPosition(ee);
    const rl::math::Transform::ConstTranslationPart p = T.translation();
    const rl::math::Vector3 abc =
        T.rotation().eulerAngles(2, 1, 0).reverse() * rl::math::RAD2DEG;

    out.x_mm = p.x() * 1000.0;
    out.y_mm = p.y() * 1000.0;
    out.z_mm = p.z() * 1000.0;
    out.a_deg = abc.x();
    out.b_deg = abc.y();
    out.c_deg = abc.z();
    return true;
}

#endif // GET_FLANGE_POSE_MM_ABC_H
