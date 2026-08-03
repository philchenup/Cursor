# glog + Console Operation Logging (MainWindow)

`src/mainwindow.cpp` records software operation logs with **glog** and mirrors every entry to the UI console via `ui->console->print` at the matching severity.

## Dual-write helpers

```cpp
void MainWindow::logInfo(const QString& msg);     // LOG_INFO  + LOG(INFO)
void MainWindow::logWarning(const QString& msg);  // LOG_WARNING + LOG(WARNING)
void MainWindow::logError(const QString& msg);    // LOG_ERROR + LOG(ERROR)
```

All button/action triggers and feedback results go through these helpers so file logs and on-screen console stay in sync.

## Covered events

- Menu / toolbar actions (Open, Save, Edit, View, Tools, Theme, Language, …)
- Camera / robot / communication buttons and status feedback
- RunOnce / config load / auto-calib / record-clear
- Normalized error and warning messages

## Initialization (required in `main`)

```cpp
#include <glog/logging.h>

int main(int argc, char* argv[])
{
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = "./logs";
    // ...
}
```

Link against glog, for example:

```cmake
find_package(glog REQUIRED)
target_link_libraries(your_target PRIVATE glog::glog)
```
