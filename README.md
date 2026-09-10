# Cursor

## computeTwoPointPoses：Y 朝世界 +Z，X 与焊缝平行

Z 仍是邻域法线均值。Y = Z × X 之后，若 `Y·world_Z < 0` 则取反，保证与世界 +Z 夹角 ≤ 90°。再 `X = Y × Z`：与起点—终点平行，方向随 Y，不要求与起点→终点同向。右手系 `X × Y = Z` 不变。

```cpp
Eigen::Vector3f y = z.cross(x_in);
if (y.dot(Eigen::Vector3f::UnitZ()) < 0.f)
    y = -y;
y.normalize();
Eigen::Vector3f x = y.cross(z).normalized();
```
