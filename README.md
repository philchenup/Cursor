# Cursor

## Cartesian control buttons

Wire XYZABC +/- buttons to `OperationalModel` via `setData` (same as table spinbox), not robot communicator pose writes.

See `snippets/README.md`, `snippets/MainWindow_cartButtons.cpp`, and
`snippets/MainWindow_sendConfigToKuka.cpp`.
