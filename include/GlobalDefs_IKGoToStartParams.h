/**
 * Patch fragment for GlobalDefs.h — replace the existing IKGoToStartParams.
 *
 * q_target_start is removed: doGoToStart now discovers the start joint
 * solution (including rail) from q_home + startPoint TCP via IK.
 */
#pragma once

// Keep this aligned with the project's GlobalDefs.h DiscretePoint / rl includes.
struct IKGoToStartParams
{
    rl::math::Vector q_home;             // Home 关节角（含地轨）
    DiscretePoint    startPoint;         // 焊接起点（mergedTraj.front()）TCP
    rl::math::Transform T_flange_to_tcp;

    double baseUpDistance = 50.0;        // 接近点相对焊点的 Base 抬升量（模型单位）
    double jointStepRad = 0.5 * M_PI / 180.0;
    double railStepLen = 5.0;
    double cartStepLen = 5.0;
    double railWindow = 100.0;           // 求解接近姿态时地轨软约束窗口（同 doSolve 约定，约 TCP.Y）
    int    timeoutMs = 500;
};
