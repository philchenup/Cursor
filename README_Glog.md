# glog Operation Logging (MainWindow)

`src/mainwindow.cpp` records software operation logs with [glog](https://github.com/google/glog).

## What is logged

- **Button / action triggers**: menu actions, camera/robot/comm buttons, RunOnce, config loaders, etc.
- **Feedback results**: connect/disconnect, enable status, vision pose output, file save/load outcomes.
- **Errors / warnings**: normalized messages for invalid input, open/parse failures, and ignored operations.

UI console (`ui->console->print`) is kept for on-screen feedback. glog writes the persistent operation trail.

## Initialization (required in `main`)

```cpp
#include <glog/logging.h>

int main(int argc, char* argv[])
{
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = "./logs";          // or your log directory
    FLAGS_alsologtostderr = true;      // optional
    // ... create QApplication / MainWindow
}
```

Link against glog in CMake / qmake, for example:

```cmake
find_package(glog REQUIRED)
target_link_libraries(your_target PRIVATE glog::glog)
```

## Helpers in `mainwindow.cpp`

- `GLogInfo` / `GLogWarning` / `GLogError` — thin wrappers around `LOG(INFO|WARNING|ERROR)`.
