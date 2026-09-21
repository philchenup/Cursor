/**
 * Patch fragment for GlobalDefs.h — replace the existing IKReturnHomeParams.
 *
 * doReturnHome:
 *   1) along TCP -Z retreat tcpStepBack → point A (IK, rail locked)
 *   2) from A: arm staging (J5→0) → rail to home → J5 → -90° / q_home
 */
#pragma once

struct IKReturnHomeParams
{
    rl::math::Vector q_current;          // 当前关节角（含地轨）
    rl::math::Vector q_home;             // Home 关节角（含地轨）
    rl::math::Transform T_flange_to_tcp;

    double tcpStepBack = 100.0;          // 沿 TCP -Z 后退距离（点 A）
    double jointStepRad = 0.5 * M_PI / 180.0;
    double railStepLen = 10.0;           // TCP 后退 / 地轨移动插值步长
    int    timeoutMs = 500;
};
