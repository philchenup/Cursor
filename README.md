# Cursor

## computeTwoPointPoses：X 与焊缝平行，Y 朝世界 +Z

- **Z**：邻域法线均值，不变
- **X**：由 `weld_dir = 终点 − 起点` 投到垂直于 Z 的平面，与焊缝平行；Y 朝上时只取 ±，不要求与起点→终点同向
- **Y**：`Y = Z × X`，`Y·world_Z < 0` 时 X、Y 一起取反

```cpp
Eigen::Vector3f weld_dir = t1 - t0;
Eigen::Vector3f x = weld_dir - z * z.dot(weld_dir);
x.normalize();
Eigen::Vector3f y = z.cross(x);
if (y.dot(Eigen::Vector3f::UnitZ()) < 0.f) {
    y = -y;
    x = -x;
}
```
