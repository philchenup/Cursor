# doReturnHome: TCP -Z retreat, then interpolate from retracted pose

## I/O

Unchanged: `IKReturnHomeParams` in → `finished_return` / `failed` / `aborted` out.

## Behavior

1. **TCP -Z retreat** by `tcpBackDistance` (default 100) → `q_retracted`.
   - Default `railStepLen=10` ⇒ **first ~10 trajectory points** are this segment.
   - Uses **incremental Jacobian IK** (no `JacobianInverseKinematics::solve` / no random restart).
   - Rail DOF frozen via `dq(0)=0` (no ultra-tight joint-limit lock).
   - Rejects a waypoint if joint jump from the previous seed is too large.
2. **Subsequent interpolation starts from `q_retracted`**:
   - `q_retracted` → staging home (rail fixed, J5 = 0)
   - rail → `q_home(0)`
   - J5 0 → -90°, final point = `q_home`
