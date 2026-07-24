# doGoToStart without `q_target_start`

## Goal

`IKWorker::doGoToStart` previously required a precomputed start joint solution
(`q_target_start`, typically `m_jointTrajectory[0]`). That created a dependency
on a prior weld IK pass. The function now only needs:

- `q_home` — current / home joints (including rail)
- `startPoint` — weld-start TCP (`mergedTraj.front()`)
- `T_flange_to_tcp` and step / timeout parameters

## Trajectory logic

1. **Discover approach pose** — soft-constrain rail near `startPoint` world Y
   (same convention as `doSolve`), IK-solve TCP above the weld by
   `baseUpDistance`, seed from home with rail ≈ Y. This yields arm joints and
   `railTarget` without `q_target_start`.
2. **Arm joint interpolate** — move arm to approach configuration while rail
   stays at home.
3. **Rail translate** — slide rail `railHome → railTarget` with arm fixed.
4. **Cartesian descend** — linearly IK-descend from approach TCP to
   `startPoint`, rail locked.

Also fixed descent segment count to use `abs(approachUp) / cartStepLen`
(previously a negative `approachUp` collapsed the descent to 1 sample).

## Integration

1. Replace `IKGoToStartParams` in `GlobalDefs.h` with
   `include/GlobalDefs_IKGoToStartParams.h` (removes `q_target_start`, adds
   optional `railWindow`).
2. Merge `tool/IKWorker.cpp` / `tool/IKWorker.h`.
3. Update `MainWindow::Trajectory` as in `src/mainwindow.cpp` (drop
   `p.q_target_start = m_jointTrajectory[0]`).
