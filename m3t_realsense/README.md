# M3T RealSense 单物体跟踪（Windows C++）

对应 [pym3t `run_realsense_example.py`](https://github.com/agimus-project/pym3t/blob/master/examples/run_realsense_example.py)，用 [M3T](https://github.com/DLR-RM/3DObjectTracking/tree/master/M3T) 的 C++ API 在 Windows 上跑同样的 RealSense 实时跟踪。

M3T **不用 PCL**。OpenCV 和 librealsense 已经装好的前提下，还需要 **Eigen3、GLEW、GLFW**（没有的话 CMake 会 FetchContent 拉取 GLEW / GLFW / Eigen）。

## 依赖

| 库 | 用途 |
| --- | --- |
| OpenCV 4 | 图像、可视化、纹理特征 |
| Intel RealSense SDK 2 (`realsense2`) | `RealSenseColorCamera` / `RealSenseDepthCamera` |
| Eigen 3 | 位姿（`Transform3fA`） |
| GLEW + GLFW + OpenGL | M3T 离屏渲染 |
| OpenMP | M3T 并行 |
| PCL（可选） | 仅用来定位 Eigen 头文件 |

物体网格必须是 Wavefront **`.obj`**，文件名 **`{body_name}.obj`**。首次运行会生成 `tmp/{body_name}_region_model.bin` 等稀疏视点模型，可能较慢。

## 配置与编译（MSVC / VS 2019+）

在 `x64 Native Tools` 或已配置好 OpenCV / RealSense 的终端里：

```bat
cd m3t_realsense
mkdir build && cd build
cmake .. -G "Visual Studio 17 2022" -A x64 -DCMAKE_BUILD_TYPE=Release ^
  -DOpenCV_DIR=C:\opencv\build ^
  -Drealsense2_DIR=C:\Program Files (x86)\Intel RealSense SDK 2.0\lib\cmake\realsense2
cmake --build . --config Release
```

已克隆 `3DObjectTracking` 时，跳过 git clone：

```bat
cmake .. -DM3T_SOURCE_DIR=D:\src\3DObjectTracking
```

`run_realsense_example.exe` 和 `m3t.dll` 输出在 `build/bin/`。

## 运行

彩色 + 区域：

```bat
.\bin\Release\run_realsense_example.exe --use_region -b obj_000014 -m D:\models
```

彩色 + 深度（对应 Python `--use_region --use_depth --measure_occlusions --use_depth_viewer`）：

```bat
.\bin\Release\run_realsense_example.exe --use_region --use_depth --measure_occlusions --use_depth_viewer -b obj_000014 -m D:\models
```

参数与 Python 脚本一致：`-b/--body_name`、`-m/--models_dir`、`--scale_geometry`（默认 `0.001`，毫米网格）、`--tmp_dir`、`--use_region`、`--use_depth`、`--use_texture`、`--use_depth_viewer`、`--model_occlusions`、`--measure_occlusions`。额外 `--print_pose` 会每帧打印 `body2world` 4x4。

默认初始位姿与 Python 相同：物体约在相机前方 0.556 m，并绕 X 轴再转 0.2 rad。把物体放到这个粗略位置，或在窗口里按 **D** 重新套用该位姿、按 **X** 开始跟踪。

窗口快捷键（M3T `Tracker`）：

- `Q` 退出
- `D` 检测（套用 `StaticDetector` 初始位姿）
- `X` 检测并开始跟踪
- `T` 开始跟踪
- `S` 停止跟踪
