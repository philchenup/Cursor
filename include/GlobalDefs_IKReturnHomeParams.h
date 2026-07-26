/**
 * Patch fragment for GlobalDefs.h — replace the existing IKReturnHomeParams.
 *
 * doReturnHome no longer performs TCP -Z retreat; tcpBackDistance is kept for
 * API compatibility only. railStepLen is used for rail translation staging.
 */
#pragma once

struct IKReturnHomeParams
{
    rl::math::Vector q_current;          // 当前关节角（含地轨）
    rl::math::Vector q_home;             // Home 关节角（含地轨）
    rl::math::Transform T_flange_to_tcp; // 保留字段（本流程未使用）

    double tcpBackDistance = 100.0;      // 保留字段（本流程未使用）
    double jointStepRad = 0.5 * M_PI / 180.0;
    double railStepLen = 10.0;           // 地轨移动插值步长
    int    timeoutMs = 500;              // 保留字段（本流程未使用）
};
