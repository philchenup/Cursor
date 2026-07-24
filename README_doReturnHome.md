# doReturnHome: TCP -Z back-off only (with near-home shortcut)

## Behavior

1. If current TCP translation distance to Home **< `tcpBackDistance` (100)**:
   **direct** joint-space interpolate `q_current → q_home` (all axes including rail),
   then finish. No TCP retreat, no J5/rail staging.
2. Otherwise:
   - TCP -Z retreat `tcpBackDistance`, IK every `railStepLen`
   - Joint interpolate to staging home (J5 = 0, rail fixed)
   - Rail translate to home with `railStepLen`
   - J5 0 → -90°

No Base/World lift. No separate `baseUpDistance` / `cartStepLen`.
