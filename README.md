# STL → 点云（PCL）

使用 Point Cloud Library 读取 STL 三角网格，表面采样为点云，并按 Z 轴着色可视化。

## 依赖

- PCL >= 1.8（需包含 `common`、`io`、`visualization`，以及 VTK 支持）
- CMake >= 3.10
- C++14 编译器

Ubuntu 安装示例：

```bash
sudo apt update
sudo apt install -y build-essential cmake libpcl-dev
```

## 编译

```bash
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

## 运行

```bash
./stl_to_pointcloud <model.stl> [采样点数]
```

参数说明：

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `model.stl` | 输入 STL 模型路径 | 必填 |
| 采样点数 | 表面随机采样点数 | `50000` |

示例：

```bash
./stl_to_pointcloud ../models/bunny.stl 80000
```

程序会：

1. 用 `pcl::io::loadPolygonFileSTL` 读取 STL
2. 按三角形面积加权，在网格表面均匀采样生成点云
3. 将点云保存为 `output_cloud.pcd`
4. 打开可视化窗口：黑底、Z 轴伪彩色、XYZ 坐标轴

关闭窗口或按 `q` 退出。

## 代码说明

核心流程见 `src/stl_to_pointcloud.cpp`：

- **读取**：`pcl::io::loadPolygonFileSTL` → `pcl::PolygonMesh`
- **采样**：在每个三角形内用重心坐标随机取点（面积加权），得到稠密表面点云
- **可视化**：`PointCloudColorHandlerGenericField` 按 `"z"` 字段着色 + `addCoordinateSystem`
