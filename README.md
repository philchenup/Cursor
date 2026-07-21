# PCL 边缘点云提取

使用 Point Cloud Library 读取点云，估计法向量，并通过边界估计提取边缘点云。

## 依赖

- PCL >= 1.8（`common`、`io`、`features`、`search`、`visualization`）
- CMake >= 3.10
- C++14

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
./extract_edge_cloud <input.pcd|ply> [k邻域] [角度阈值(度)] [体素边长] [线程数]
```

| 参数 | 说明 | 默认值 |
|------|------|--------|
| 输入文件 | `.pcd` 或 `.ply` 点云 | 必填 |
| k邻域 | 法向量与边界估计的邻域点数 | `30` |
| 角度阈值 | 判定边界的空隙角度（度），越小越敏感 | `90` |
| 体素边长 | `>0` 时先 VoxelGrid 降采样 | `0`（不降采样） |
| 线程数 | OpenMP 并行线程数 | 硬件并发数 |

示例（推荐加速参数）：

```bash
# 稠密点云：先降采样 + 多线程
./extract_edge_cloud ../models/cloud.pcd 20 90 0.005 8
```

程序会：

1. 读取点云
2. （可选）VoxelGrid 降采样
3. 用 `NormalEstimationOMP` 并行估计法向量
4. 用多线程 `BoundaryEstimation` 检测边缘点
5. 保存边缘点云为 `edge_cloud.pcd`，并打印各阶段耗时
6. 可视化：灰色=工作点云，红色=边缘点云

## 加速策略

复杂度近似 `O(N · k · log N)`，优先减小 `N`，再并行化：

| 优先级 | 手段 | 说明 |
|--------|------|------|
| 1 | VoxelGrid 降采样 | 点数降到 1/5～1/20，通常收益最大 |
| 2 | `NormalEstimationOMP` + `setNumberOfThreads` | 法向量/边界估计吃满多核 |
| 3 | 复用同一棵 KdTree | 避免法向量与边界估计各建一次树 |
| 4 | 适当减小 k | 邻域查询更便宜，边缘略变锐利 |
| 5 | 有序点云改用 Organized 接口 | 深度相机数据可换 `OrganizedEdgeFromNormals`，无需 KdTree |

调参建议：

- 边缘过少：减小角度阈值（如 `60`），或增大 k
- 边缘过多/噪声：增大角度阈值（如 `120`），或增大体素边长
- 仍慢：先确认 PCL 是否以 OpenMP 编译（`NormalEstimationOMP` 才真正并行）
