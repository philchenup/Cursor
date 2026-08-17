#include "ShapeWorldLocation.h"

#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>
#include <PrsMgr_PresentableObject.hxx>

namespace {

gp_Pnt OriginOf(const gp_Trsf& trsf)
{
    return gp_Pnt(0.0, 0.0, 0.0).Transformed(trsf);
}

bool BoundingBoxCenter(const TopoDS_Shape& shape, gp_Pnt& center)
{
    Bnd_Box box;
    BRepBndLib::Add(shape, box);
    if (box.IsVoid()) {
        return false;
    }

    Standard_Real xmin = 0.0;
    Standard_Real ymin = 0.0;
    Standard_Real zmin = 0.0;
    Standard_Real xmax = 0.0;
    Standard_Real ymax = 0.0;
    Standard_Real zmax = 0.0;
    box.Get(xmin, ymin, zmin, xmax, ymax, zmax);
    center = gp_Pnt(0.5 * (xmin + xmax), 0.5 * (ymin + ymax), 0.5 * (zmin + zmax));
    return true;
}

} // namespace

gp_Trsf GetWorldTransformation(const TopoDS_Shape& shape)
{
    gp_Trsf trsf;
    if (shape.IsNull()) {
        return trsf;
    }
    trsf = shape.Location().Transformation();
    return trsf;
}

gp_Trsf GetWorldTransformation(const AIS_Shape* ais)
{
    gp_Trsf world;
    if (ais == nullptr) {
        return world;
    }

    // 按值拷贝 LocalTransformation，再沿父节点向外 PreMultiply。
    // 结果等价于 parentN * ... * parent1 * local。
    world = ais->LocalTransformation();
    Handle(PrsMgr_PresentableObject) parent = ais->Parent();
    while (!parent.IsNull()) {
        gp_Trsf parentLocal = parent->LocalTransformation();
        world.PreMultiply(parentLocal);
        parent = parent->Parent();
    }
    return world;
}

gp_Pnt GetWorldPosition(const TopoDS_Shape& shape)
{
    return OriginOf(GetWorldTransformation(shape));
}

gp_Pnt GetWorldPosition(const AIS_Shape* ais)
{
    return OriginOf(GetWorldTransformation(ais));
}

gp_Pnt GetWorldCenter(const TopoDS_Shape& shape)
{
    gp_Pnt center;
    if (shape.IsNull() || !BoundingBoxCenter(shape, center)) {
        return GetWorldPosition(shape);
    }
    return center;
}

gp_Pnt GetWorldCenter(const AIS_Shape* ais)
{
    if (ais == nullptr) {
        return gp_Pnt(0.0, 0.0, 0.0);
    }

    const gp_Trsf world = GetWorldTransformation(ais);
    gp_Pnt center;
    if (!BoundingBoxCenter(ais->Shape(), center)) {
        return OriginOf(world);
    }
    return center.Transformed(world);
}

gp_Pnt ToWorldPoint(const TopoDS_Shape& shape, const gp_Pnt& localPoint)
{
    return localPoint.Transformed(GetWorldTransformation(shape));
}

gp_Pnt ToWorldPoint(const AIS_Shape* ais, const gp_Pnt& localPoint)
{
    return localPoint.Transformed(GetWorldTransformation(ais));
}
