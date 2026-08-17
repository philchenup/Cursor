#ifndef SCALE_AIS_SHAPE_H
#define SCALE_AIS_SHAPE_H

#include <AIS_Shape.hxx>

/**
 * @brief 将 AIS_Shape 缩小 1000 倍，并返回一个新的 AIS_Shape。
 *
 * 以坐标原点 (0, 0, 0) 为缩放中心，对几何体施加均匀缩放 1/1000。
 * 若原始对象带有局部变换，会一并合成后再应用到几何上。
 * 新对象会尽量复制颜色、材质、透明度和显示模式。
 *
 * @param ais 原始 AIS_Shape 指针
 * @return 缩小后的新 AIS_Shape 指针；ais 为空或几何为空时返回 nullptr
 *
 * @note 返回的对象继承 Standard_Transient，建议立即用 Handle 接管：
 *       Handle(AIS_Shape) scaled = ScaleAISShapeBy1000(ais);
 */
AIS_Shape* ScaleAISShapeBy1000(AIS_Shape* ais);

inline Handle(AIS_Shape) ScaleAISShapeBy1000(const Handle(AIS_Shape)& ais)
{
    return ScaleAISShapeBy1000(ais.get());
}

#endif // SCALE_AIS_SHAPE_H
