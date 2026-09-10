# Cursor

## computeTwoPointPoses 右手系

旧 `makePose` 用 `Y = X × Z`、`X = Z × Y`，得到 **X × Y = −Z**（左手系），Y 指向 `Z × X` 的反方向。

改为：

- **X**：起点→终点，投影到垂直于 Z 的平面
- **Z**：邻域法线均值
- **Y = Z × X**，再 **X = Y × Z**，保证 **X × Y = Z**，`det(R) = +1`

```cpp
Eigen::Vector3f z = z_in.normalized();
Eigen::Vector3f y = z.cross(x_in);          // Y = Z × X
if (y.squaredNorm() < 1e-12f)
    y = z.cross((std::fabs(z.z()) < 0.9f) ? Eigen::Vector3f::UnitZ()
                                          : Eigen::Vector3f::UnitX());
y.normalize();
Eigen::Vector3f x = y.cross(z).normalized(); // X = Y × Z
```
