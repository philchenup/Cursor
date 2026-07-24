/**
 * Patch fragment for GlobalDefs.h — replace the existing IKReturnHomeParams.
 *
 * TCP -Z back-off is interpolated with railStepLen (no separate cart/base-up params).
 */
#pragma once

struct IKReturnHomeParams
{
    rl::math::Vector q_current;          // 当前关节角（含地轨）
    rl::math::Vector q_home;             // Home 关节角（含地轨）
    rl::math::Transform T_flange_to_tcp;

    double tcpBackDistance = 100.0;      // 沿 TCP -Z 后退距离
    double jointStepRad = 0.5 * M_PI / 180.0;
    double railStepLen = 10.0;           // TCP 后退 / 地轨移动插值步长
    int    timeoutMs = 500;
};
