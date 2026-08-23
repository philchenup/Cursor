# FilterReg (C++ port of neka-nat/probreg)

只保留 [probreg](https://github.com/neka-nat/probreg) 中的 **Rigid FilterReg**（`pt2pt` / `pt2pl`）。
不包含 CPD、GMMReg、SVR、GMMTree、BCPD 或可变形 FilterReg。

算法对应 `probreg/filterreg.py` 的 `RigidFilterReg`：

1. **E-step**：permutohedral lattice 高斯滤波估计对应（`m0/m1/m2/nx`）
2. **M-step**：`pt2pt` 用加权 Kabsch，`pt2pl` 用 twist 线性化

论文：Gao & Tedrake, *FilterReg*, CVPR 2019.

## 文件

```
filterreg/
  filterreg.h / filterreg.cpp     # Rigid FilterReg
  permutohedral.h / .cpp          # 高维高斯滤波
  main.cpp                        # PCL 读 PLY + demo
  data/*.ply                      # bunny / cloud_0 / cloud_1
```

依赖：CMake ≥ 3.10、C++14、Eigen3（`libeigen3-dev`）、PCL（`libpcl-dev`，仅 demo 读 PLY）。

## 构建与运行

```bash
sudo apt-get install -y libeigen3-dev libpcl-dev
cd filterreg
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER=g++
cmake --build build -j

./build/filterreg_demo --test
./build/filterreg_demo pt2pt ./build/bunny.ply ./build/bunny.ply
./build/filterreg_demo pt2pl ./build/cloud_0.ply ./build/cloud_1.ply
```

`RunPt2Pt` / `RunPt2Pl` 接收两个 `N×3` 点云，坐标单位为 **毫米**。算法内部换算成米再求解，输出的平移 `t` 和 `sigma2` 仍是毫米。

配准结果写出 `matched_source.ply` / `matched_target.ply`（毫米）。
