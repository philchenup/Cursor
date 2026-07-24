# doReturnHome: TCP -Z back-off only

## Behavior

1. Along TCP -Z retreat `tcpBackDistance` (default 100), IK samples every `railStepLen`
2. Joint interpolate to staging home (J5 = 0, rail fixed)
3. Rail translate to home with `railStepLen`
4. J5 0 → -90°

No Base/World lift. No separate `baseUpDistance` / `cartStepLen`.
