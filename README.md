# 基于点云的边缘提取算法（C++ / PCL）

实现两种经典边缘提取算法：

| 方法 | 适用场景 | 判据 |
|------|----------|------|
| `boundary` | 外轮廓、孔洞边界 | 切平面邻域方位角最大间隙 > 阈值 |
| `curvature` | 棱边、尖锐几何特征 | PCA 曲率 σ = λ₀/(λ₀+λ₁+λ₂) ≥ 阈值 |

## 依赖

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
./extract_edge_cloud <input.pcd|ply> [method] [k] [threshold] [voxel] [threads]
```

| 参数 | 说明 | 默认 |
|------|------|------|
| method | `boundary` 或 `curvature` | `boundary` |
| k | 邻域点数 | `30` |
| threshold | boundary=角度(度)；curvature=曲率 | `90` / `0.1` |
| voxel | 体素边长，`>0` 先降采样 | `0` |
| threads | OpenMP 线程数 | 硬件并发 |

```bash
# 轮廓/孔洞边界
./extract_edge_cloud ../models/cloud.pcd boundary 30 90 0.005 8

# 棱边/尖锐特征
./extract_edge_cloud ../models/cloud.pcd curvature 30 0.1 0.005 8
```

输出：`edge_cloud.pcd`（红=边缘，灰=原云）

## 算法说明

### 1. Boundary（切平面角度空隙）

1. 估计点法向量 **n**
2. 将 k 近邻投影到切平面，计算方位角并排序
3. 最大相邻角间隙 Δθ_max > 阈值 ⇒ 边界点

### 2. Curvature（PCA 曲率）

1. 对 k 近邻构建协方差矩阵并特征值分解
2. σ = λ₀ / (λ₀+λ₁+λ₂)
3. σ ≥ 阈值 ⇒ 边缘点（高曲率/棱边）

## 加速

- VoxelGrid 降采样减小 N
- `NormalEstimationOMP` 并行法向量
- 复用同一棵 KdTree
