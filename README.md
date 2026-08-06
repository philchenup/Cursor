# PCL + VTK 点云工程视图（正视 / 俯视 / 左视）

用 **PCL** 加载点云，**VTK Camera** 设置工程视角并打开正交投影。提供两个程序：

| 程序 | 说明 |
|------|------|
| `pcl_vtk_view_buttons` | **三个按钮**切换正视 / 俯视 / 左视（推荐） |
| `pcl_vtk_ortho_views` | 键盘切换 / 四视口对照 |

## 三按钮版（推荐）

窗口上方三个 `QPushButton`，点击即切换正交工程视图：

```
┌──────────┬──────────┬──────────┐
│  正视图  │  俯视图  │  左视图  │
└──────────┴──────────┴──────────┘
│                                │
│         VTK 点云窗口            │
│                                │
└────────────────────────────────┘
```

按钮回调核心逻辑：

```cpp
connect(btn_front, &QPushButton::clicked, [&]{ setView(ViewType::Front); });
connect(btn_top,   &QPushButton::clicked, [&]{ setView(ViewType::Top); });
connect(btn_left,  &QPushButton::clicked, [&]{ setView(ViewType::Left); });

void setView(ViewType view) {
  // 1. 按视图算相机 pos / focal / up
  cam->SetPosition(...);
  cam->SetFocalPoint(...);
  cam->SetViewUp(...);
  // 2. 正交投影
  cam->ParallelProjectionOn();
  cam->SetParallelScale(radius * 1.1);
  renderWindow->Render();
}
```

坐标系：**Z 向上**，+X 右，+Y 前。

| 按钮 | 相机相对中心 | ViewUp |
|------|-------------|--------|
| 正视图 | `(0, -d, 0)` | `(0,0,1)` |
| 俯视图 | `(0, 0, +d)` | `(0,1,0)` |
| 左视图 | `(-d, 0, 0)` | `(0,0,1)` |

## 依赖

```bash
sudo apt install build-essential cmake libpcl-dev \
  qtbase5-dev libvtk9-dev libvtk9-qt-dev
# 包名因发行版而异；需带 Qt 支持的 VTK（GUISupportQt / QVTKOpenGLNativeWidget）
```

- PCL ≥ 1.12
- VTK（含 `QVTKOpenGLNativeWidget`）
- Qt5 Widgets
- CMake ≥ 3.10，C++14

## 编译与运行

```bash
mkdir -p build && cd build
cmake ..
make -j$(nproc)

# 三按钮界面（无文件则用演示长方体）
./pcl_vtk_view_buttons
./pcl_vtk_view_buttons /path/to/cloud.pcd

# 键盘 / 四视口版
./pcl_vtk_ortho_views
./pcl_vtk_ortho_views cloud.ply --single
```

## 文件

```
CMakeLists.txt
src/pcl_vtk_view_buttons.cpp   # 三按钮 Qt 版
src/pcl_vtk_ortho_views.cpp    # 键盘 / 四视口版
README.md
```
