# AGENTS.md

## Cursor Cloud specific instructions

### What this repo is
This repo is **not a standalone app**. It is a small C++/Qt module (`src/SocketComm.{h,cpp}`
+ `src/mainwindow_initialize_comm.cpp`) that adds TCP Client / TCP Server / UDP socket
communication (`SocketWorker`) to a larger Qt robotics desktop application built on the
[Robotics Library](https://github.com/roboticslibrary/rl) (`rl::hal::Socket`). See
`README_SocketComm.md` for integration details. The parent app (`mainwindow.h`, the `.ui`
file, `ct::LOG_*`) lives outside this repo.

### Environment (already provisioned in the VM snapshot)
- `g++` 13 / `cmake`, Qt5 dev (`qtbase5-dev`, `moc` at `/usr/lib/qt5/bin/moc`), Eigen, Boost.
- Robotics Library built from source and installed to `/usr/local`
  (headers at `/usr/local/include/rl-0.7.0/rl`, lib `/usr/local/lib/librlhal.so`).

### Gotchas
- **`/usr/bin/c++` is `clang++`** (via update-alternatives) and fails to link libstdc++
  (`cannot find -lstdc++`). Always build with **`g++`/`gcc`**. For CMake pass
  `-DCMAKE_C_COMPILER=gcc -DCMAKE_CXX_COMPILER=g++`.
- The code includes `<hal/Socket.h>` (not `<rl/hal/Socket.h>`), so add BOTH include roots:
  `-I/usr/local/include/rl-0.7.0/rl -I/usr/local/include/rl-0.7.0`.
- `SocketComm.h` contains `Q_OBJECT`, so it must be run through `moc` before compiling.
- **`src/mainwindow_initialize_comm.cpp` cannot compile standalone** — it needs the parent
  app's `mainwindow.h` and generated `ui_*.h`. Only `SocketComm.{h,cpp}` is buildable in
  isolation.
- Rebuilding the Robotics Library is NOT needed on startup; it persists in the snapshot.
  The update script only refreshes the linker cache (`ldconfig`).

### Build + smoke-test the socket module in isolation
There is no build system in the repo. To verify `SocketComm` compiles and works, build it
against Qt5 + RL (a standalone `SocketWorker` TCP+UDP smoke test was validated this way):

```bash
RLINC=/usr/local/include/rl-0.7.0
/usr/lib/qt5/bin/moc -I src -I "$RLINC/rl" -I "$RLINC" src/SocketComm.h -o /tmp/moc_SocketComm.cpp
g++ -std=c++17 -fPIC -Wall -Wextra $(pkg-config --cflags Qt5Core) \
    -I src -I "$RLINC/rl" -I "$RLINC" -I /usr/include/eigen3 \
    src/SocketComm.cpp /tmp/moc_SocketComm.cpp <your_main.cpp> \
    $(pkg-config --libs Qt5Core) -L/usr/local/lib -lrlhal -lpthread -o /tmp/sc_test
```

### Lint / test
No lint/test config ships in the repo. Use a `g++ -std=c++17 -Wall -Wextra -fsyntax-only`
pass over `src/SocketComm.cpp` (with the include flags above) as a lightweight lint.
