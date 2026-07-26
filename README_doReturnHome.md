# doReturnHome: TCP step-back then A → Home

## Params (`IKReturnHomeParams`)

| Field | Role |
|-------|------|
| `q_current` / `q_home` | Start / Home joint vectors (incl. rail) |
| `T_flange_to_tcp` | Flange→TCP |
| `tcpStepBack` | Retreat distance along TCP **-Z** (default 100) |
| `jointStepRad` | Arm joint interpolation step |
| `railStepLen` | TCP retreat & rail translate step |
| `timeoutMs` | Per-point IK timeout |

Signals unchanged: `finished_return(jointTrajectory, ratio)`, `failed`, `aborted`, `progress`.

## Path

1. **TCP -Z retreat** `tcpStepBack` with IK every `railStepLen` → **point A** (rail locked at `q_current(0)`).
2. **A → Home** joint-space plan:
   - Arm to staging Home (rail fixed, J5 = 0)
   - Rail translate to `q_home(0)`
   - J5 0 → -90°, snap to `q_home`
