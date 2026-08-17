#include "ScaleAISShape.h"

#include <AIS_ColoredShape.hxx>
#include <AIS_InteractiveContext.hxx>
#include <AIS_Shape.hxx>

// 用法示例：缩放后颜色（含 AIS_ColoredShape 分面色）会跟到新对象上
void ReplaceWithScaledShape(const Handle(AIS_InteractiveContext)& context,
                            const Handle(AIS_Shape)& ais,
                            float factor)
{
    Handle(AIS_Shape) scaledAis = ScaleAISShape(ais, factor);
    if (scaledAis.IsNull()) {
        return;
    }

    context->Remove(ais, Standard_False);
    context->Display(scaledAis, Standard_True);
}

// 对应 ReadModel::ScaleAISShape 的调用方式（裸指针）
AIS_Shape* ScaleLoadedModel(AIS_Shape* loadShape, float factor)
{
    return ScaleAISShape(loadShape, factor);
}
