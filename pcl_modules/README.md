# ct_pcl_modules

将 CloudTool 的 `filters` / `features` / `segmentation` / `surface` 以及 `Cloud` 基类改为**不依赖 Qt** 的标准 C++ 库，仅需 **PCL** 即可在 Windows 上用 CMake 编译。

原 Qt 信号槽已改为函数的**输出参数**：

```cpp
ct::Filters filters;
filters.setInputCloud(cloud);

ct::Cloud::Ptr filtered;
float time_ms = 0.f;
filters.VoxelGrid(0.05f, 0.05f, 0.05f, filtered, time_ms);
```

## 目录

```
pcl_modules/
  CMakeLists.txt
  include/base/{cloud,common,exports}.h
  include/modules/{filters,features,segmentation,surface}.h
  src/base/{cloud,common}.cpp
  src/modules/{filters,features,segmentation,surface}.cpp
  examples/example.cpp
```

## Windows 编译

依赖：

- Visual Studio 2019/2022（x64）
- [PCL All-in-One](https://github.com/PointCloudLibrary/pcl/releases)（建议 1.11 及以上）
- 与 PCL 配套的 Eigen / Boost / FLANN / Qhull（安装包一般已带）

在 `pcl_modules` 目录下：

```bat
mkdir build
cd build

cmake -S .. -B . -G "Visual Studio 17 2022" -A x64 ^
  -DPCL_DIR="C:/Program Files/PCL 1.14.1/cmake"

cmake --build . --config Release
```

Visual Studio 2019 把生成器改成 `"Visual Studio 16 2019"`。

也可使用：

```bat
cmake -S .. -B . -G "Visual Studio 17 2022" -A x64 ^
  -DPCL_ROOT="C:/Program Files/PCL 1.14.1"
```

或把 PCL 根目录加入 `CMAKE_PREFIX_PATH`。

编译产物：

- 静态库：`ct_pcl_modules.lib`（默认）
- 示例程序：`bin/Release/ct_pcl_example.exe`

运行示例：

```bat
.\bin\Release\ct_pcl_example.exe
```

若运行时提示缺少 DLL，把 PCL / VTK / OpenNI2 的 `bin` 目录加入 `PATH`，例如：

```bat
set PATH=C:\Program Files\PCL 1.14.1\bin;%PATH%
```

CMake 只查找计算模块（`common` / `io` / `filters` / `features` / `segmentation` / `surface` 等），不链接 PCL 可视化或 Qt。

## 可选 CMake 选项

| 选项 | 默认 | 说明 |
| --- | --- | --- |
| `CT_BUILD_SHARED` | `OFF` | `ON` 时编译为 DLL |
| `CT_BUILD_EXAMPLES` | `ON` | 编译示例程序 |
| `WITH_OPENMP` | `ON` | 启用 OpenMP |

## 接口变化

| 原来（Qt） | 现在（标准 C++） |
| --- | --- |
| `QObject` / `Q_OBJECT` / `signals` / `slots` / `emit` | 普通类 + 输出参数 |
| `QString` | `std::string` |
| `QFileInfo` | `std::string` 路径 + 文件大小 |
| `emit filterResult(cloud, time)` | `void Foo(..., Cloud::Ptr& cloud_out, float& time)` |
| `emit segmentationResult(id, clouds, time, coef)` | `void Foo(..., std::vector<Cloud::Ptr>& clouds, float& time[, ModelCoefficients::Ptr& cofe])` |
| `emit surfaceResult(id, mesh, time)` | `void Foo(..., PolygonMesh::Ptr& mesh, float& time)` |
| `emit featureResult(id, feature, time)` | `void Foo(..., FeatureType::Ptr& feature, float& time)` |

点云 `id` 仍可通过 `cloud->id()` 读取，不再单独作为信号参数传出。
