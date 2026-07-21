# PCA / Sigma 点云边缘提取（PCL 1.12 + C++14）

基于邻域协方差特征值曲率的边缘提取，修复原代码问题并适配 PCL 1.12。

## 算法

对每个点取 K 近邻，构建协方差矩阵并特征值分解（λs ≤ λm ≤ λl）：

```
Sigma = λs / (λs + λm + λl)
```

`Sigma` 较大 → 局部弯曲/棱边显著 → 标为边缘（红色）。

判定阈值：`MinSigma + N * (MaxSigma - MinSigma) / 256`（默认 N=6）。

## 相对原代码的主要修正

| 问题 | 修正 |
|------|------|
| 重复 `#include`、未使用变量 | 清理头文件与死代码 |
| `new[]` 未 `delete` | 改用 `std::vector` |
| 手工排序特征值 | 直接用 `SelfAdjointEigenSolver` 升序结果 |
| `MinD` 初值设为点数 | 改为 `numeric_limits<double>::max()` |
| 特征值除零风险 | 对特征值之和做保护 |
| `CloudViewer` | 改为 PCL 1.12 常用的 `PCLVisualizer` |
| 硬编码路径 | 支持命令行参数 |
| 点类型 `PointXYZRGBA` | 改为 `PointXYZRGB`（着色足够且更通用） |

## 依赖

- PCL **1.12**
- Eigen3
- CMake ≥ 3.10，C++14

```bash
# Ubuntu 示例（版本因源而异，需确保为 1.12）
sudo apt install build-essential cmake libpcl-dev libeigen3-dev
```

## 编译

```bash
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

## 运行

```bash
./extract_edge_cloud <input.ply|pcd> [K邻域] [sigma倍数]
```

```bash
./extract_edge_cloud ../data/Edge.ply 10 6
```

| 参数 | 默认 | 说明 |
|------|------|------|
| K邻域 | 10 | 协方差邻域点数 |
| sigma倍数 | 6 | 阈值 = Min + 倍数×step；越大边缘越少 |

输出：

- `edge_cloud.ply` — 仅边缘点
- `cloud_colored.ply` — 全图着色（khaki / 红）
