# doReturnHome: TCP -Z back-off Cartesian interpolation

## Change

Step 1 of `doReturnHome` previously solved a single IK goal for the full
`tcpBackDistance` retreat along TCP -Z. It now interpolates that retreat in
Cartesian space with step `cartStepLen` (default **10 mm**).

For `tcpBackDistance = 100` and `cartStepLen = 10`, this yields 10 IK samples
along TCP -Z before the Base +Z lift.

## Params (`IKReturnHomeParams`)

- `tcpBackDistance` default `100.0`
- `cartStepLen` added, default `10.0`

## Integration

1. Merge `include/GlobalDefs_IKReturnHomeParams.h` into project `GlobalDefs.h`
2. Merge `tool/IKWorker.cpp` / `tool/IKWorker.h`
3. Update `MainWindow::robotGoHome` as in `src/mainwindow.cpp`
