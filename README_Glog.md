# glog + Console Operation Logging (MainWindow)

Complete sources (with `GlobalDefs.h` preserved):

- `src/mainwindow.h` / `src/mainwindow.cpp`
- mirrors: `src/MainWindow.h` / `src/MainWindow.cpp`

## Dual-write helpers

Every operation log goes through:

```cpp
void MainWindow::logInfo(const QString& msg);     // ui->console->print(LOG_INFO)  + LOG(INFO)
void MainWindow::logWarning(const QString& msg);  // ui->console->print(LOG_WARNING) + LOG(WARNING)
void MainWindow::logError(const QString& msg);    // ui->console->print(LOG_ERROR) + LOG(ERROR)
```

## Covered

- Menu / toolbar / button triggers (`Action triggered` / `Button clicked`)
- Camera / laser / robot / trajectory / weld feedback
- Normalized error and warning messages

## Setup in `main`

```cpp
#include <glog/logging.h>
google::InitGoogleLogging(argv[0]);
FLAGS_log_dir = "./logs";
```

```cmake
find_package(glog REQUIRED)
target_link_libraries(your_target PRIVATE glog::glog)
```
