#include "ShapeWorldLocation.h"

#include <iostream>

// 用法示例：打印 AIS_Shape / TopoDS_Shape 在世界坐标系中的位置
void PrintWorldLocation(const Handle(AIS_Shape)& ais)
{
    const gp_Trsf worldTrsf = GetWorldTransformation(ais);
    const gp_Pnt origin = GetWorldPosition(ais);
    const gp_Pnt center = GetWorldCenter(ais);

    std::cout << "AIS origin (world): "
              << origin.X() << ", " << origin.Y() << ", " << origin.Z()
              << "\n";
    std::cout << "AIS bbox center (world): "
              << center.X() << ", " << center.Y() << ", " << center.Z()
              << "\n";

    const TopoDS_Shape& shape = ais->Shape();
    const gp_Pnt shapeOrigin = GetWorldPosition(shape);
    std::cout << "TopoDS_Shape Location origin: "
              << shapeOrigin.X() << ", " << shapeOrigin.Y() << ", " << shapeOrigin.Z()
              << "\n";

    (void)worldTrsf;
}
