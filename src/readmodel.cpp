#include "readmodel.h"

#include <Interface_Static.hxx>
#include <RWStl.hxx>
#include <Poly_Triangulation.hxx>
#include <BRep_Builder.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Compound.hxx>
#include <OSD_Path.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <Aspect_TypeOfLine.hxx>
#include <Prs3d_Drawer.hxx>
#include <Prs3d_LineAspect.hxx>
#include <STEPCAFControl_Reader.hxx>
#include <TDF_Label.hxx>
#include <TDF_LabelSequence.hxx>
#include <TopExp_Explorer.hxx>
#include <XCAFApp_Application.hxx>
#include <XCAFDoc_ColorTool.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>

namespace {

Handle(TDocStd_Document) newXcafDocument()
{
    Handle(XCAFApp_Application) app = XCAFApp_Application::GetApplication();
    Handle(TDocStd_Document) doc;
    app->NewDocument("MDTV-XCAF", doc);
    return doc;
}

bool tryGetColor(const Handle(XCAFDoc_ColorTool)& colorTool,
                 const TDF_Label& label,
                 const TopoDS_Shape& shape,
                 Quantity_Color& color)
{
    if (colorTool.IsNull()) {
        return false;
    }
    if (colorTool->GetColor(label, XCAFDoc_ColorSurf, color)) {
        return true;
    }
    if (colorTool->GetColor(label, XCAFDoc_ColorGen, color)) {
        return true;
    }
    if (colorTool->GetColor(label, XCAFDoc_ColorCurv, color)) {
        return true;
    }
    if (!shape.IsNull()) {
        if (colorTool->GetColor(shape, XCAFDoc_ColorSurf, color)) {
            return true;
        }
        if (colorTool->GetColor(shape, XCAFDoc_ColorGen, color)) {
            return true;
        }
    }
    return false;
}

void applyLabelColors(const Handle(XCAFDoc_ShapeTool)& shapeTool,
                      const Handle(XCAFDoc_ColorTool)& colorTool,
                      const TDF_Label& label,
                      const Handle(AIS_ColoredShape)& ais)
{
    if (shapeTool.IsNull() || colorTool.IsNull() || ais.IsNull() || label.IsNull()) {
        return;
    }

    TopoDS_Shape shape = shapeTool->GetShape(label);
    Quantity_Color color;
    if (!shape.IsNull() && tryGetColor(colorTool, label, shape, color)) {
        ais->SetCustomColor(shape, color);
    }

    if (shapeTool->IsAssembly(label)) {
        TDF_LabelSequence comps;
        shapeTool->GetComponents(label, comps);
        for (Standard_Integer i = 1; i <= comps.Length(); ++i) {
            applyLabelColors(shapeTool, colorTool, comps.Value(i), ais);
        }
        return;
    }

    if (shapeTool->IsReference(label)) {
        TDF_Label ref;
        if (shapeTool->GetReferredShape(label, ref)) {
            applyLabelColors(shapeTool, colorTool, ref, ais);
        }
    }

    TDF_LabelSequence subs;
    shapeTool->GetSubShapes(label, subs);
    for (Standard_Integer i = 1; i <= subs.Length(); ++i) {
        applyLabelColors(shapeTool, colorTool, subs.Value(i), ais);
    }

    if (!shape.IsNull()) {
        for (TopExp_Explorer exp(shape, TopAbs_FACE); exp.More(); exp.Next()) {
            const TopoDS_Shape face = exp.Current();
            Quantity_Color faceColor;
            if (colorTool->GetColor(face, XCAFDoc_ColorSurf, faceColor) ||
                colorTool->GetColor(face, XCAFDoc_ColorGen, faceColor)) {
                ais->SetCustomColor(face, faceColor);
            }
        }
    }
}

Handle(AIS_ColoredShape) shadeDefaults(const Handle(AIS_ColoredShape)& ais,
                                       const Quantity_Color& color)
{
    if (ais.IsNull()) {
        return ais;
    }
    ais->SetColor(color);
    ais->SetDisplayMode(AIS_Shaded);
    ais->Attributes()->SetFaceBoundaryDraw(Standard_True);
    ais->Attributes()->SetFaceBoundaryAspect(
        new Prs3d_LineAspect(Quantity_NOC_BLACK, Aspect_TOL_SOLID, 1.0));
    return ais;
}

} // namespace

ReadModel::ReadModel()
{

}

TopoDS_Shape ScaleShape(const TopoDS_Shape& inputShape, double scaleFactor)
{
    // 1. 检查输入Shape是否有效
    if (inputShape.IsNull()) {
        return TopoDS_Shape();
    }

    // 2. 创建缩放变换矩阵
    gp_Trsf scaleTransform;
    // 以坐标原点(0,0,0)为缩放中心，等比例缩放x/y/z
    scaleTransform.SetScale(gp_Pnt(0, 0, 0), scaleFactor);

    // 3. 应用变换到Shape上
    // Mode=Standard_True 表示创建新的拓扑结构（推荐，不修改原Shape）
    BRepBuilderAPI_Transform transformTool(inputShape, scaleTransform, Standard_True);
    transformTool.Build();

    // 4. 获取缩放后的Shape
    TopoDS_Shape scaledShape = transformTool.Shape();

    // 5. 验证缩放结果
    if (scaledShape.IsNull()) {
        return TopoDS_Shape();
    }

    return scaledShape;
}

TopoDS_Shape ReadModel::readStlModel(Standard_CString filePath)
{
    /*TopoDS_Shape shape_Stl;
	StlAPI_Reader aReader_Stl;
	
	bool isread = aReader_Stl.Read(shape_Stl, filePath);

	return shape_Stl;*/

    Handle(Poly_Triangulation) tri = RWStl::ReadFile(OSD_Path(filePath));

    TopoDS_Shape shape_Stl;
    if (tri.IsNull())
        return shape_Stl;   // 读取失败返回空 shape

    TopoDS_Face face;
    BRep_Builder builder;
    builder.MakeFace(face, tri);

    shape_Stl = face;
    return shape_Stl;
}

TopoDS_Shape ReadModel::readStepModel(Standard_CString filePath) {

	STEPControl_Reader stepReader;
	IFSelect_ReturnStatus readStatus = stepReader.ReadFile(filePath);
	if (readStatus != IFSelect_RetDone) {
		return TopoDS_Shape(); 
	}
	stepReader.TransferRoots();
	return stepReader.OneShape();
}

ReadModel::ColoredModel ReadModel::readStepModelWithColors(Standard_CString filePath)
{
    ColoredModel out;
    if (filePath == nullptr) {
        return out;
    }

    Handle(TDocStd_Document) doc = newXcafDocument();
    STEPCAFControl_Reader reader;
    reader.SetColorMode(true);
    reader.SetNameMode(true);
    reader.SetLayerMode(true);

    if (reader.ReadFile(filePath) != IFSelect_RetDone || !reader.Transfer(doc)) {
        // 回退到原有无颜色读取
        out.shape = readStepModel(filePath);
        return out;
    }

    Handle(XCAFDoc_ShapeTool) shapeTool =
        XCAFDoc_DocumentTool::ShapeTool(doc->Main());
    Handle(XCAFDoc_ColorTool) colorTool =
        XCAFDoc_DocumentTool::ColorTool(doc->Main());

    TDF_LabelSequence freeShapes;
    shapeTool->GetFreeShapes(freeShapes);
    if (freeShapes.Length() == 0) {
        return out;
    }

    if (freeShapes.Length() == 1) {
        out.shape = shapeTool->GetShape(freeShapes.Value(1));
    } else {
        TopoDS_Compound compound;
        BRep_Builder builder;
        builder.MakeCompound(compound);
        for (Standard_Integer i = 1; i <= freeShapes.Length(); ++i) {
            builder.Add(compound, shapeTool->GetShape(freeShapes.Value(i)));
        }
        out.shape = compound;
    }

    TDF_LabelSequence colorLabels;
    colorTool->GetColors(colorLabels);
    out.hasColors = colorLabels.Length() > 0;
    out.xcafDoc = doc;
    return out;
}

Handle(AIS_ColoredShape) ReadModel::makeDisplayShape(
    const ColoredModel& model,
    const Quantity_Color& fallbackColor)
{
    if (model.shape.IsNull()) {
        return Handle(AIS_ColoredShape)();
    }

    Handle(AIS_ColoredShape) ais = new AIS_ColoredShape(model.shape);
    shadeDefaults(ais, fallbackColor);

    if (model.hasColors && !model.xcafDoc.IsNull()) {
        Handle(XCAFDoc_ShapeTool) shapeTool =
            XCAFDoc_DocumentTool::ShapeTool(model.xcafDoc->Main());
        Handle(XCAFDoc_ColorTool) colorTool =
            XCAFDoc_DocumentTool::ColorTool(model.xcafDoc->Main());

        TDF_LabelSequence freeShapes;
        shapeTool->GetFreeShapes(freeShapes);
        for (Standard_Integer i = 1; i <= freeShapes.Length(); ++i) {
            applyLabelColors(shapeTool, colorTool, freeShapes.Value(i), ais);
        }
    }

    return ais;
}

Handle(AIS_ColoredShape) ReadModel::makeDisplayShape(
    const TopoDS_Shape& shape,
    const Quantity_Color& color)
{
    if (shape.IsNull()) {
        return Handle(AIS_ColoredShape)();
    }
    Handle(AIS_ColoredShape) ais = new AIS_ColoredShape(shape);
    return shadeDefaults(ais, color);
}

void ReadModel::writeStepModel(TopoDS_Shape shape, Standard_CString filePath)
{
	if (shape.IsNull()) return;

	Interface_Static::SetCVal("write.step.unit", "M");
	STEPControl_Writer write;

	IFSelect_ReturnStatus stat1 = write.Transfer(shape, STEPControl_AsIs);

	IFSelect_ReturnStatus stat2 = write.Write(filePath);

	return;
}

void ReadModel::writeStlModel(TopoDS_Shape shape, Standard_CString filePath) {
	if (shape.IsNull()) return;

	StlAPI_Writer stlWriter;

	stlWriter.Write(shape, filePath);

	return;
}
