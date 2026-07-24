# doReturnHome: TCP -Z back-off only (with near-home shortcut)

## Behavior

1. If current TCP translation distance to Home **< `tcpBackDistance` (100)**:
   skip TCP retreat; start joint/rail/J5 interpolation from `q_current`
2. Otherwise: along TCP -Z retreat `tcpBackDistance`, IK every `railStepLen`
3. Joint interpolate to staging home (J5 = 0, rail fixed)
4. Rail translate to home with `railStepLen`
5. J5 0 → -90°

No Base/World lift. No separate `baseUpDistance` / `cartStepLen`.
