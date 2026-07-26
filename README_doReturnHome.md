# doReturnHome: TCP -Z back-off, then interpolate from retracted pose

## I/O

Unchanged: `IKReturnHomeParams` in, `finished_return(jointTrajectory, ratio)` / `failed` / `aborted` out.

## Behavior

Always:

1. **TCP -Z retreat** `tcpBackDistance` (default 100), IK every `railStepLen`, rail locked at `q_current(0)`.
2. From the **retracted pose** `q_retracted`:
   - Joint interpolate to staging home (rail fixed, J5 = 0)
   - Rail translate to `q_home(0)` with J5 held at 0
   - J5 0 → -90°

No near-home shortcut. Cartesian IK uses `setRandomRestarts(0)` so solutions stay continuous from the seed (avoids start jumps from random restarts).
