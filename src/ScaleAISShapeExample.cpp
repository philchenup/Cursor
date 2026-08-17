#include "ScaleAISShape.h"

#include <AIS_InteractiveContext.hxx>
#include <AIS_Shape.hxx>

// 用法示例：将已有 AIS_Shape 缩小 1000 倍后重新显示
void ReplaceWithScaledShape(const Handle(AIS_InteractiveContext)& context,
                            const Handle(AIS_Shape)& ais)
{
    Handle(AIS_Shape) scaledAis = ScaleAISShapeBy1000(ais);
    if (scaledAis.IsNull()) {
        return;
    }

    context->Remove(ais, Standard_False);
    context->Display(scaledAis, Standard_True);
}
