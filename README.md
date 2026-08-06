# PCL + VTK 点云工程视图（正视 / 俯视 / 左视）

用 **PCL** 加载与可视化点云，用底层 **VTK Camera** 设置工程视角并打开正交投影，实现 CAD 风格的正视图、俯视图、左视图等。

## 原理

`PCLVisualizer` 本身基于 VTK。切换工程视图只需两步：

1. **相机位姿**（位置 / 焦点 / 上方向）——对应正视、俯视、左视…
2. **正交投影** `vtkCamera::ParallelProjectionOn()` —— 否则仍是透视，边线会汇聚

默认坐标系：**Z 向上**，+X 右，+Y 前：

| 视图 | 相机相对物体中心 | ViewUp |
|------|------------------|--------|
| 正视 Front | `(0, -d, 0)` | `(0,0,1)` |
| 俯视 Top | `(0, 0, +d)` | `(0,1,0)` |
| 左视 Left | `(-d, 0, 0)` | `(0,0,1)` |
| 右视 Right | `(+d, 0, 0)` | `(0,0,1)` |
| 后视 Back | `(0, +d, 0)` | `(0,0,1)` |
| 等轴测 Iso | `(d,d,d)/√3` | `(0,0,1)` |

核心代码（见 `applyView`）：

```cpp
// 多视口时：每个视口单独一台相机（否则会一起转）
viewer.createViewPort(xmin, ymin, xmax, ymax, vp);
viewer.createViewPortCamera(vp);

viewer.setCameraPosition(px, py, pz, fx, fy, fz, ux, uy, uz, vp);

vtkCamera* cam = /* 该视口 ActiveCamera */;
cam->ParallelProjectionOn();
cam->SetParallelScale(bounds.radius * 1.1);  // 正交半高度
```

若你的数据是 **Y 向上**，只需改 `viewCameraPose` 里各视图的 `pos` / `up`。

## 依赖

- PCL ≥ 1.12（含 `visualization`）
- VTK（一般随 `libpcl-dev` 带上）
- CMake ≥ 3.10，C++14

```bash
sudo apt install build-essential cmake libpcl-dev
```

## 编译与运行

```bash
mkdir -p build && cd build
cmake ..
make -j$(nproc)

# 四视口（默认）：左下正视 / 左上俯视 / 右下左视 / 右上等轴测
./pcl_vtk_ortho_views
./pcl_vtk_ortho_views /path/to/cloud.pcd

# 单视口，键盘切换
./pcl_vtk_ortho_views --single
./pcl_vtk_ortho_views cloud.ply --single
```

未传文件时会生成演示长方体 + 红/绿/蓝 XYZ 轴，方便核对三视图。

### 快捷键

| 键 | 作用 |
|----|------|
| `1`~`6` | 正视 / 俯视 / 左视 / 右视 / 后视 / 等轴测 |
| `o` | 正交 ↔ 透视 |
| `q` | 退出 |

四视口模式下，`1`~`6` 只切换右上角那一格，便于和固定三视图对照。

## 文件

```
CMakeLists.txt
src/pcl_vtk_ortho_views.cpp
README.md
```
