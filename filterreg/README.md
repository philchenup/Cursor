# filterreg

将 `filterreg` / `permutohedral` / `pcd_utils` 在 **Windows** 上用 CMake + Visual Studio 编译。

| 目标 | 源文件 | 依赖 |
| --- | --- | --- |
| `filterreg.lib` | `filterreg.cpp` + `permutohedral.cpp` | **Eigen3** |
| `pcd_utils.lib`（可选） | `pcd_utils.cpp` | Eigen、**PCL**、**OpenCV**、**OpenMP**、**small_gicp**、仓库中的 `pcl_modules`（`ct::Cloud`） |

## Windows 依赖

- Visual Studio 2019 / 2022（x64）
- [Eigen](https://eigen.tuxfamily.org/)（可用 PCL All-in-One 自带的 3rdParty/Eigen3）
- 编译 `pcd_utils` 时还需要：
  - [PCL All-in-One](https://github.com/PointCloudLibrary/pcl/releases)（建议 1.11+）
  - [OpenCV](https://opencv.org/releases/)
  - [small_gicp](https://github.com/koide3/small_gicp) 源码或安装前缀
  - 本仓库的 `pcl_modules`（提供 `cloud.h` / `ct::Cloud`）

## 编译（仅 FilterReg，只要 Eigen）

```bat
cd filterreg
cmake -S . -B build -G "Visual Studio 17 2022" -A x64 ^
  -DEIGEN3_INCLUDE_DIR="C:/Program Files/PCL 1.14.1/3rdParty/Eigen3/include" ^
  -DFILTERREG_BUILD_PCD_UTILS=OFF

cmake --build build --config Release
```

若 Eigen 带 CMake 包：

```bat
cmake -S . -B build -G "Visual Studio 17 2022" -A x64 ^
  -DEigen3_DIR="C:/Program Files/Eigen3/share/eigen3/cmake" ^
  -DFILTERREG_BUILD_PCD_UTILS=OFF
```

## 编译（含 pcd_utils）

先克隆 small_gicp（只需头文件即可）：

```bat
git clone https://github.com/koide3/small_gicp.git C:\src\small_gicp
```

```bat
cd filterreg
cmake -S . -B build -G "Visual Studio 17 2022" -A x64 ^
  -DPCL_DIR="C:/Program Files/PCL 1.14.1/cmake" ^
  -DOpenCV_DIR="C:/opencv/build" ^
  -DSMALL_GICP_DIR="C:/src/small_gicp"

cmake --build build --config Release
```

Visual Studio 2019 把生成器改成 `"Visual Studio 16 2019"`。

产物：

- `build/Release/filterreg.lib`
- `build/Release/pcd_utils.lib`（开启 `FILTERREG_BUILD_PCD_UTILS` 时）
- `build/bin/Release/example_filterreg.exe`
- `build/bin/Release/example_pcd_utils.exe`

运行示例若缺 DLL，把 PCL / OpenCV 的 `bin` 加入 `PATH`：

```bat
set PATH=C:\Program Files\PCL 1.14.1\bin;C:\opencv\build\x64\vc16\bin;%PATH%
.\build\bin\Release\example_filterreg.exe
```

## CMake 选项

| 选项 | 默认 | 说明 |
| --- | --- | --- |
| `FILTERREG_BUILD_PCD_UTILS` | `ON` | 编译 `pcd_utils` |
| `FILTERREG_BUILD_EXAMPLES` | `ON` | 编译示例程序 |
| `EIGEN3_INCLUDE_DIR` | 自动查找 | Eigen 头文件目录 |
| `PCL_DIR` / `PCL_ROOT` | — | PCL CMake 配置目录 |
| `OpenCV_DIR` | — | OpenCV `build` 目录 |
| `SMALL_GICP_DIR` | — | small_gicp 源码根目录（含 `include/small_gicp`） |
| `PCL_MODULES_DIR` | `../pcl_modules` | `ct::Cloud` 所在工程 |

## 头文件

```cpp
#include "filterreg.h"      // filterreg::registration
#include "permutohedral.h"  // Permutohedral 高维高斯滤波
#include "pcd_utils.h"      // 点云下采样 / FilterReg / 背景去除等
```
