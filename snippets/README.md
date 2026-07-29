# Cartesian step buttons → OperationalModel

Do **not** go through `KukaCommunicator` / `g_robotData.pose` if the goal is the same behavior as the table spinbox. The spinbox path is:

`QDoubleSpinBox::valueChanged` → `OperationalDelegate::valueChanged` → `commitData` → `setModelData` → `OperationalModel::setData` (IK + `dataChanged` → ConfigurationModel refresh).

## Changes

1. Add `OperationalModel::stepAxis(int axis, int dir, double stepSize)` (see `src/OperationalModel.*`).
2. Wire the 12 buttons as in `MainWindow_cartButtons.cpp`.

## Why this matches the spinbox

| Axis | Model row | EditRole unit | Step source (current wiring) |
|------|-----------|---------------|------------------------------|
| X/Y/Z | 0/1/2 | mm (translation) | `transStepSpinbox` → `OperationalDelegate::setSingleStep` |
| A/B/C | 3/4/5 | deg (euler) | same `stepSize` (delegate switches on `column`, which is always 0) |

`setData` already runs Jacobian IK and emits `dataChanged`, so the existing
`operationalModel → configurationModel` connection keeps joint space in sync.
