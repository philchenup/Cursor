# Open3D 点云配准算法计时对比

基于 `src.ply` / `target.ply`，对比以下 5 种配准算法的计算时间：

1. `register_fast_global`
2. `register_fpfh_ransac`
3. `register_icp_generalized`
4. `register_icp_point_to_plane`
5. `register_icp_point_to_point`

## 依赖

```bash
pip install open3d numpy
```

## 运行

将 `src.ply` 与 `target.ply` 放在当前目录后执行：

```bash
python registration_benchmark.py --no-show
```

结果会打印到终端，并保存到 `output/registration_benchmark.txt`。
