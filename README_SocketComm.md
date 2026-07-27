# Socket Communication — TCP Server / TCP Client / UDP

`ui->scoketComb` supports three modes. Drop these files into the project and wire `InitializeComm` as shown.

## Files

| File | Role |
|------|------|
| `src/SocketComm.h` | `SocketWorker` + `SocketCommMode` |
| `src/SocketComm.cpp` | TCP Client recv, TCP Server accept+recv, UDP recvfrom/sendto |
| `src/mainwindow_initialize_comm.cpp` | `MainWindow::InitializeComm()` implementation |

## Mode behavior

| Combo text | Behavior |
|------------|----------|
| **TCP Client** | `open` + `connect` to `ip:port`, then recv/send on the connected socket |
| **TCP Server** | `open` + `bind` + `listen` on `ip` (or `0.0.0.0` if IP empty) `:port`, `accept` in worker thread, then recv/send; after client disconnect, waits for the next client |
| **UDP** | Bind local `0.0.0.0:port`, `sendto` remote `ip:port`, `recvfrom` updates peer for replies |

## MainWindow members

Replace the old UDP-only members (`m_udpSocket`, `m_udpMode`, `m_udpAddress`) with:

```cpp
#include "SocketComm.h"

SocketWorker* m_worker_comm = nullptr;
QThread*      m_thread_comm = nullptr;
bool          m_comm_connected = false;
void          setConnected(bool connected); // sets m_comm_connected + button text / UI state
```

## Build

Add `src/SocketComm.cpp` and `src/mainwindow_initialize_comm.cpp` to the target (or merge `InitializeComm` into your existing `mainwindow.cpp`). Ensure Robotics Library `rl::hal::Socket` and Qt Network/Core are linked as before.
