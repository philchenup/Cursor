#include "ScaleAISShape.h"

#include <BRepBuilderAPI_Transform.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <Quantity_Color.hxx>
#include <TopoDS_Shape.hxx>

namespace {
constexpr double kScaleFactor = 1.0 / 1000.0;

void CopyDisplayAttributes(AIS_Shape* source, AIS_Shape* target)
{
    Quantity_Color color;
    if (source->HasColor()) {
        source->Color(color);
        target->SetColor(color);
    }

    if (source->HasMaterial()) {
        target->SetMaterial(source->Material());
    }

    if (source->IsTransparent()) {
        target->SetTransparency(source->Transparency());
    }

    target->SetDisplayMode(source->DisplayMode());
    target->SetWidth(source->Width());
}
} // namespace

AIS_Shape* ScaleAISShapeBy1000(AIS_Shape* ais)
{
    if (ais == nullptr) {
        return nullptr;
    }

    const TopoDS_Shape& sourceShape = ais->Shape();
    if (sourceShape.IsNull()) {
        return nullptr;
    }

    gp_Trsf scaleTrsf;
    scaleTrsf.SetScale(gp_Pnt(0.0, 0.0, 0.0), kScaleFactor);

    // 若原对象已有显示变换，先应用再缩小，保证结果与屏幕上看到的一致
    gp_Trsf totalTrsf = scaleTrsf;
    if (ais->HasTransformation()) {
        totalTrsf = scaleTrsf * ais->Transformation();
    }

    BRepBuilderAPI_Transform transformer(sourceShape, totalTrsf, Standard_True);
    if (!transformer.IsDone()) {
        return nullptr;
    }

    AIS_Shape* scaledAis = new AIS_Shape(transformer.Shape());
    CopyDisplayAttributes(ais, scaledAis);
    return scaledAis;
}
