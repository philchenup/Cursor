# M3T Windows CMake（DLL + GLEW）

把本目录的 `CMakeLists.txt`、`src/CMakeLists.txt` 和 `cmake/` 覆盖到 [DLR-RM/3DObjectTracking](https://github.com/DLR-RM/3DObjectTracking) 的 `M3T/` 树中，再配置编译。

## 修了什么

1. **Windows 不再只产出 `.lib`**  
   原先 `add_library(m3t STATIC ...)` 只能生成静态库。默认 `BUILD_SHARED_LIBS=ON`，并打开 `CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS`。MSVC 会得到：
   - `bin/m3t.dll`（运行时）
   - `lib/m3t.lib`（导入库）
   - 示例程序也写到 `bin/`，和 DLL 放在一起

   若仍要静态库：`-DBUILD_SHARED_LIBS=OFF`。

2. **能找到 GLEW**  
   不再默认 `GLEW_STATIC`（那会去找 `glew32s.lib`，官方 zip 的共享库是 `glew32.lib`）。查找顺序：
   - CMake CONFIG（vcpkg 等）
   - MODULE / 手动路径（`GLEW_ROOT`、`GLEW_DIR`、`GLEW_INCLUDE_DIR(S)`、`GLEW_LIBRARY(IES)`）
   - 找不到时 FetchContent 拉取 [glew-cmake 2.2.0](https://github.com/Perlmint/glew-cmake)

   官方 Windows zip 的目录即可：

   ```
   <GLEW_ROOT>/include/GL/glew.h
   <GLEW_ROOT>/lib/Release/x64/glew32.lib
   <GLEW_ROOT>/bin/Release/x64/glew32.dll
   ```

## Windows 配置示例

```bat
cd M3T
cmake -S . -B build -G "Visual Studio 17 2022" -A x64 ^
  -DBUILD_SHARED_LIBS=ON ^
  -DUSE_AZURE_KINECT=OFF ^
  -DUSE_REALSENSE=ON ^
  -DEigen3_DIR="C:/eigen-3.4.0/share/eigen3/cmake" ^
  -DOpenCV_DIR="C:/opencv/build" ^
  -DGLEW_ROOT="C:/Tools/glew-2.1.0"
cmake --build build --config Release
```

产物在 `build/bin/`（`m3t.dll` 与 exe）和 `build/lib/`（`m3t.lib`）。

若未安装 GLEW、也不传 `GLEW_ROOT`，配置阶段会自动下载编译 GLEW。关掉自动下载：`-DM3T_FETCH_GLEW=OFF`。
