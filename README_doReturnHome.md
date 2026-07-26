# doReturnHome: TCP -Z retreat, then interpolate from retracted pose

## I/O

Unchanged: `IKReturnHomeParams` in → `finished_return` / `failed` / `aborted` out.

## Behavior

1. **TCP -Z retreat** by `tcpBackDistance` (default 100), rail locked at `q_current(0)` → `q_retracted`.
2. **Subsequent interpolation starts from `q_retracted`** (not from `q_current`):
   - `q_retracted` → staging home (rail fixed, J5 = 0)
   - rail → `q_home(0)` (arm held from staging / retracted chain)
   - J5 0 → -90°, final point = `q_home`

Cartesian IK uses `setRandomRestarts(0)` for continuous retreat from the seed.
