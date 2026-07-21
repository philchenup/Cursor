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
./extract_edge_cloud <input.pcd|ply> [k邻域] [角度阈值(度)]
```

| 参数 | 说明 | 默认值 |
|------|------|--------|
| 输入文件 | `.pcd` 或 `.ply` 点云 | 必填 |
| k邻域 | 法向量与边界估计的邻域点数 | `30` |
| 角度阈值 | 判定边界的空隙角度（度），越小越敏感 | `90` |

示例：

```bash
./extract_edge_cloud ../models/cloud.pcd 30 90
```

程序会：

1. 读取点云
2. 用 `NormalEstimation` 估计法向量
3. 用 `BoundaryEstimation` 检测边缘点
4. 保存边缘点云为 `edge_cloud.pcd`
5. 可视化：灰色=原始点云，红色=边缘点云

## 原理简述

对每个点，将其 k 邻域投影到切平面，统计邻域点在切平面上的角度分布。若存在大于阈值的角度空隙，则该点位于点云边界/边缘上。

调参建议：

- 边缘过少：减小角度阈值（如 `60`），或增大 k
- 边缘过多/噪声：增大角度阈值（如 `120`），或先对点云下采样/滤波
