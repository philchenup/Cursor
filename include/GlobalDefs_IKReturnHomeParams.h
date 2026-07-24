/**
 * Patch fragment for GlobalDefs.h — replace the existing IKReturnHomeParams.
 *
 * cartStepLen: TCP -Z back-off Cartesian interpolation step (default 10 mm).
 */
#pragma once

struct IKReturnHomeParams
{
    rl::math::Vector q_current;          // 当前关节角（含地轨）
    rl::math::Vector q_home;             // Home 关节角（含地轨）
    rl::math::Transform T_flange_to_tcp;

    double tcpBackDistance = 100.0;      // 沿 TCP -Z 后退距离
    double baseUpDistance = 100.0;       // Base/World 抬升距离
    double jointStepRad = 0.5 * M_PI / 180.0;
    double railStepLen = 5.0;
    double cartStepLen = 10.0;           // TCP 后退笛卡尔插值步长
    int    timeoutMs = 500;
};
