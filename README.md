# Cursor

## computeTwoPointPoses：X 就是起点→终点

不要把焊缝投到垂直于 Z 的平面。X 直接是 `normalize(终点 − 起点)`；Y 朝世界 +Z 时只把 X、Y 一起取反，X 仍在起点—终点连线上。Z 仍是邻域法线。

```cpp
Eigen::Vector3f x = weld_dir.normalized(); // 起点→终点，不投影
Eigen::Vector3f y = z.cross(x);
if (y.dot(Eigen::Vector3f::UnitZ()) < 0.f) {
    y = -y;
    x = -x;
}
```
