# doReturnHome: joint / rail / J5 staging (no TCP retreat)

## I/O

Unchanged: `IKReturnHomeParams` in → `finished_return` / `failed` / `aborted` out.
`tcpBackDistance` and `T_flange_to_tcp` remain in the params struct for API compatibility but are unused.

## Behavior

Starts from `q_current` (no TCP -Z back-off):

1. Joint interpolate to staging home (rail fixed at `q_current(0)`, J5 = 0)
2. Rail → `q_home(0)` (arm held at staging)
3. J5 0 → -90°, final point = `q_home`
