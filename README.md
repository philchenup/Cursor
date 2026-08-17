# Cursor

## gpTrsfToRl

按两个库的官方分量接口转换（不要拷 `Value()`/`VectorialPart()` 进 `T(i,j)`）：

| | OpenCASCADE `gp_Trsf` | Robotics Library `rl::math::Transform` |
|---|---|---|
| 类型 | 相似变换：旋转 + 比例 + 平移 | `Eigen::Transform<Real, 3, Eigen::Affine>` |
| 旋转 | `GetRotation()` → `gp_Quaternion` | `linear()` ← `Quaternion(w,x,y,z).toRotationMatrix()` |
| 比例 | `ScaleFactor()` | 乘进 `linear()` |
| 平移 | `TranslationPart()` | `translation()` |

Eigen 四元数构造是 `(w, x, y, z)`，OCCT 构造是 `(x, y, z, w)`，必须用 `q.W()/X()/Y()/Z()`。

```cpp
rl::math::Transform tf = gpTrsfToRl(ais->Transformation());
body->setFrame(tf);

// 反向（OCCT 会正交化）
const rl::math::Quaternion q(tf.rotation());
const rl::math::Vector3 t = tf.translation();
gp_Trsf T0;
T0.SetTransformation(gp_Quaternion(q.x(), q.y(), q.z(), q.w()), gp_Vec(t.x(), t.y(), t.z()));
```

## ScaleAISShapeBy1000

将 OpenCASCADE 的 `AIS_Shape*` 缩小 1000 倍，并返回一个新的 `AIS_Shape*`。

```cpp
#include "ScaleAISShape.h"

AIS_Shape* ais = /* 已有对象 */;
AIS_Shape* scaled = ScaleAISShapeBy1000(ais);

// 推荐用 Handle 接管返回值，避免泄漏
Handle(AIS_Shape) scaledHandle = ScaleAISShapeBy1000(ais);
```
