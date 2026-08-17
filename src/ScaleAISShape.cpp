#include "ScaleAISShape.h"

#include <AIS_ColoredDrawer.hxx>
#include <AIS_ColoredShape.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <Prs3d_Drawer.hxx>
#include <Prs3d_LineAspect.hxx>
#include <Quantity_Color.hxx>
#include <Standard_Failure.hxx>
#include <TopoDS_Shape.hxx>
#include <TopTools_ListOfShape.hxx>

namespace {

constexpr float kScaleFactor1000 = 1.0f / 1000.0f;

TopoDS_Shape MappedSubshape(BRepBuilderAPI_Transform& transformer,
                            const TopoDS_Shape& sub)
{
    if (sub.IsNull() || transformer.IsDeleted(sub)) {
        return TopoDS_Shape();
    }

    const TopTools_ListOfShape& modified = transformer.Modified(sub);
    if (!modified.IsEmpty()) {
        return modified.First();
    }

    try {
        const TopoDS_Shape mapped = transformer.ModifiedShape(sub);
        if (!mapped.IsNull()) {
            return mapped;
        }
    } catch (Standard_Failure const&) {
    }

    return TopoDS_Shape();
}

void CopyDisplayAttributes(AIS_Shape* source, AIS_Shape* target)
{
    Quantity_Color color;
    source->Color(color);
    // 无论 HasColor() 与否都写过去：STEP 导入后常用 SetColor 作为底色，
    // 缩放后若丢失会变成 context 默认色。
    target->SetColor(color);

    if (source->HasMaterial()) {
        target->SetMaterial(source->Material());
    }

    if (source->IsTransparent()) {
        target->SetTransparency(source->Transparency());
    }

    target->SetDisplayMode(source->DisplayMode());
    target->SetWidth(source->Width());

    if (!source->Attributes().IsNull() && !target->Attributes().IsNull()) {
        target->Attributes()->SetFaceBoundaryDraw(
            source->Attributes()->FaceBoundaryDraw());
        if (source->Attributes()->HasOwnFaceBoundaryAspect()
            && !source->Attributes()->FaceBoundaryAspect().IsNull()) {
            target->Attributes()->SetFaceBoundaryAspect(
                source->Attributes()->FaceBoundaryAspect());
        }
    }
}

void CopyCustomColors(AIS_ColoredShape* source,
                      AIS_ColoredShape* target,
                      BRepBuilderAPI_Transform& transformer)
{
    const AIS_DataMapOfShapeDrawer& map = source->CustomAspectsMap();
    for (AIS_DataMapOfShapeDrawer::Iterator it(map); it.More(); it.Next()) {
        const Handle(AIS_ColoredDrawer)& srcDrawer = it.Value();
        if (srcDrawer.IsNull()) {
            continue;
        }

        const TopoDS_Shape mapped = MappedSubshape(transformer, it.Key());
        if (mapped.IsNull()) {
            continue;
        }

        if (srcDrawer->IsHidden()) {
            target->CustomAspects(mapped)->SetHidden(Standard_True);
        }
        if (srcDrawer->HasOwnColor()) {
            target->SetCustomColor(mapped, srcDrawer->Color());
        }
        if (srcDrawer->HasOwnTransparency()) {
            target->SetCustomTransparency(mapped, srcDrawer->Transparency());
        }
        if (srcDrawer->HasOwnWidth() && !srcDrawer->LineAspect().IsNull()) {
            target->SetCustomWidth(mapped, srcDrawer->LineAspect()->Aspect()->Width());
        }
    }
}

} // namespace

AIS_Shape* ScaleAISShape(AIS_Shape* ais, float factor)
{
    if (ais == nullptr || factor == 0.0f) {
        return nullptr;
    }

    const TopoDS_Shape& sourceShape = ais->Shape();
    if (sourceShape.IsNull()) {
        return nullptr;
    }

    gp_Trsf scaleTrsf;
    scaleTrsf.SetScale(gp_Pnt(0.0, 0.0, 0.0), static_cast<Standard_Real>(factor));

    gp_Trsf totalTrsf = scaleTrsf;
    if (ais->HasTransformation()) {
        totalTrsf = scaleTrsf * ais->Transformation();
    }

    BRepBuilderAPI_Transform transformer(sourceShape, totalTrsf, Standard_True);
    if (!transformer.IsDone()) {
        return nullptr;
    }

    // 用 AIS_ColoredShape 承接，才能把分面/零件颜色绑到缩放后的新拓扑上。
    // AIS_ColoredShape 继承 AIS_Shape，调用方可继续按 AIS_Shape* 使用。
    AIS_ColoredShape* scaledAis = new AIS_ColoredShape(transformer.Shape());
    CopyDisplayAttributes(ais, scaledAis);

    if (ais->IsKind(STANDARD_TYPE(AIS_ColoredShape))) {
        CopyCustomColors(static_cast<AIS_ColoredShape*>(ais), scaledAis, transformer);
    }

    return scaledAis;
}

AIS_Shape* ScaleAISShapeBy1000(AIS_Shape* ais)
{
    return ScaleAISShape(ais, kScaleFactor1000);
}
