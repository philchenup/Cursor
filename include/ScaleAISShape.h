#ifndef SCALE_AIS_SHAPE_H
#define SCALE_AIS_SHAPE_H

#include <AIS_Shape.hxx>

/**
 * @brief 以原点为中心等比缩放 AIS_Shape，并返回新对象。
 *
 * 几何通过 BRepBuilderAPI_Transform 真正缩小/放大；若原对象带局部变换，
 * 会先合成再应用到 BRep。
 *
 * 颜色会一并带到新对象上：
 * - 整体颜色 / 材质 / 透明度 / 线宽 / 显示模式
 * - 若 ais 实际是 AIS_ColoredShape（STEP 分面着色），子形状颜色会按
 *   变换后的新拓扑重新绑定，避免缩放后变成单色
 *
 * 返回类型仍是 AIS_Shape*，多色时底层是 AIS_ColoredShape。
 *
 * @param ais    原始 AIS_Shape 指针（可为 AIS_ColoredShape）
 * @param factor 缩放系数，例如 0.001f 表示缩小 1000 倍（mm → m）
 * @return 缩放后的新对象；ais 为空、几何为空或 factor==0 时返回 nullptr
 *
 * @note 返回对象继承 Standard_Transient，建议立即用 Handle 接管：
 *       Handle(AIS_Shape) scaled = ScaleAISShape(ais, 0.001f);
 *
 * 也可直接作为 ReadModel::ScaleAISShape 的实现使用。
 */
AIS_Shape* ScaleAISShape(AIS_Shape* ais, float factor);

/** 缩小 1000 倍的便捷封装，等价于 ScaleAISShape(ais, 0.001f)。 */
AIS_Shape* ScaleAISShapeBy1000(AIS_Shape* ais);

inline Handle(AIS_Shape) ScaleAISShape(const Handle(AIS_Shape)& ais, float factor)
{
    return ScaleAISShape(ais.get(), factor);
}

inline Handle(AIS_Shape) ScaleAISShapeBy1000(const Handle(AIS_Shape)& ais)
{
    return ScaleAISShapeBy1000(ais.get());
}

#endif // SCALE_AIS_SHAPE_H
