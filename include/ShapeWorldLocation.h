#ifndef SHAPE_WORLD_LOCATION_H
#define SHAPE_WORLD_LOCATION_H

#include <AIS_Shape.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <TopoDS_Shape.hxx>

/**
 * @file ShapeWorldLocation.h
 *
 * 获取 TopoDS_Shape / AIS_Shape 在世界坐标系中的位姿与位置。
 *
 * 约定：
 * - TopoDS_Shape 的世界变换来自 Location()（几何自身携带的坐标系）。
 * - AIS_Shape 的世界变换 = 父节点链 × LocalTransformation（显示坐标系）。
 *   AIS_Shape::Shape() 的几何已含自身 Location；显示时再叠加上面的世界变换。
 * - 不要写 `const gp_Trsf& t = ais->Transformation()`：请按值拷贝，
 *   并用本文件中的函数合成父节点链。
 *
 * 位置有两种常用含义：
 * - GetWorldPosition：局部坐标系原点映射到世界后的点。
 * - GetWorldCenter：包围盒中心映射到世界后的点（几何“在哪儿”）。
 */

/// TopoDS_Shape 的 Location 变换；空形状返回单位变换。
gp_Trsf GetWorldTransformation(const TopoDS_Shape& shape);

/// AIS_Shape 在世界坐标系中的显示变换（父节点链 × LocalTransformation）。
/// ais 为空时返回单位变换。
gp_Trsf GetWorldTransformation(const AIS_Shape* ais);

inline gp_Trsf GetWorldTransformation(const Handle(AIS_Shape)& ais)
{
    return GetWorldTransformation(ais.get());
}

/// 局部原点 (0,0,0) 在世界坐标系中的位置。
gp_Pnt GetWorldPosition(const TopoDS_Shape& shape);
gp_Pnt GetWorldPosition(const AIS_Shape* ais);

inline gp_Pnt GetWorldPosition(const Handle(AIS_Shape)& ais)
{
    return GetWorldPosition(ais.get());
}

/// 包围盒中心在世界坐标系中的位置；空形状或空盒时退回 GetWorldPosition。
gp_Pnt GetWorldCenter(const TopoDS_Shape& shape);
gp_Pnt GetWorldCenter(const AIS_Shape* ais);

inline gp_Pnt GetWorldCenter(const Handle(AIS_Shape)& ais)
{
    return GetWorldCenter(ais.get());
}

/**
 * 将局部点变换到世界坐标系。
 *
 * - TopoDS_Shape：localPoint 在 Location 之前的局部系中。
 * - AIS_Shape：localPoint 在显示局部系中（与 Shape() 取出的几何点同一套，
 *   已含 Shape Location，再叠加上面的 AIS 世界变换）。
 *
 * 若需要 AIS 几何局部原点的世界坐标：
 *   ToWorldPoint(ais, GetWorldPosition(ais->Shape()));
 */
gp_Pnt ToWorldPoint(const TopoDS_Shape& shape, const gp_Pnt& localPoint);
gp_Pnt ToWorldPoint(const AIS_Shape* ais, const gp_Pnt& localPoint);

inline gp_Pnt ToWorldPoint(const Handle(AIS_Shape)& ais, const gp_Pnt& localPoint)
{
    return ToWorldPoint(ais.get(), localPoint);
}

#endif // SHAPE_WORLD_LOCATION_H
