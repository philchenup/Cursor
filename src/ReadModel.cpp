#include "ReadModel.h"

#include <AIS_ColoredShape.hxx>
#include <Aspect_TypeOfLine.hxx>
#include <BRep_Builder.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <Prs3d_Drawer.hxx>
#include <Prs3d_LineAspect.hxx>
#include <Quantity_Color.hxx>
#include <STEPCAFControl_Reader.hxx>
#include <STEPControl_Reader.hxx>
#include <StlAPI_Reader.hxx>
#include <TDF_Label.hxx>
#include <TDF_LabelSequence.hxx>
#include <TDocStd_Document.hxx>
#include <TopoDS_Compound.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <XCAFApp_Application.hxx>
#include <XCAFDoc_ColorTool.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>

namespace {

Handle(TDocStd_Document) newXcafDocument() {
  Handle(XCAFApp_Application) app = XCAFApp_Application::GetApplication();
  Handle(TDocStd_Document) doc;
  app->NewDocument("MDTV-XCAF", doc);
  return doc;
}

bool tryGetColor(const Handle(XCAFDoc_ColorTool)& colorTool,
                 const TDF_Label& label,
                 const TopoDS_Shape& shape,
                 Quantity_Color& color) {
  if (!colorTool.IsNull()) {
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
  }
  return false;
}

/// Walk XCAF labels and map colors onto AIS_ColoredShape sub-shapes.
void applyLabelColors(const Handle(XCAFDoc_ShapeTool)& shapeTool,
                      const Handle(XCAFDoc_ColorTool)& colorTool,
                      const TDF_Label& label,
                      const Handle(AIS_ColoredShape)& ais) {
  if (shapeTool.IsNull() || colorTool.IsNull() || ais.IsNull() || label.IsNull()) {
    return;
  }

  TopoDS_Shape shape = shapeTool->GetShape(label);
  Quantity_Color color;
  if (!shape.IsNull() && tryGetColor(colorTool, label, shape, color)) {
    ais->SetCustomColor(shape, color);
  }

  // Components (assemblies)
  if (shapeTool->IsAssembly(label)) {
    TDF_LabelSequence comps;
    shapeTool->GetComponents(label, comps);
    for (Standard_Integer i = 1; i <= comps.Length(); ++i) {
      applyLabelColors(shapeTool, colorTool, comps.Value(i), ais);
    }
    return;
  }

  // Referenced solid / shape under a component instance
  if (shapeTool->IsReference(label)) {
    TDF_Label ref;
    if (shapeTool->GetReferredShape(label, ref)) {
      applyLabelColors(shapeTool, colorTool, ref, ais);
    }
  }

  // Direct sub-shapes (solids / shells / faces with own color)
  TDF_LabelSequence subs;
  shapeTool->GetSubShapes(label, subs);
  for (Standard_Integer i = 1; i <= subs.Length(); ++i) {
    applyLabelColors(shapeTool, colorTool, subs.Value(i), ais);
  }

  // Also color individual faces looked up by shape key (common in STEP paints)
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
                                       const Quantity_Color& color) {
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

}  // namespace

TopoDS_Shape ReadModel::readStlModel(const char* filename) {
  TopoDS_Shape shape;
  if (filename == nullptr) {
    return shape;
  }

  StlAPI_Reader reader;
  if (!reader.Read(shape, filename)) {
    return TopoDS_Shape();
  }
  return shape;
}

TopoDS_Shape ReadModel::readStepModel(const char* filename) {
  if (filename == nullptr) {
    return TopoDS_Shape();
  }

  STEPControl_Reader reader;
  if (reader.ReadFile(filename) != IFSelect_RetDone) {
    return TopoDS_Shape();
  }
  reader.TransferRoots();
  return reader.OneShape();
}

ReadModel::ColoredModel ReadModel::readStepModelWithColors(const char* filename) {
  ColoredModel out;
  if (filename == nullptr) {
    return out;
  }

  Handle(TDocStd_Document) doc = newXcafDocument();
  STEPCAFControl_Reader reader;
  reader.SetColorMode(true);
  reader.SetNameMode(true);
  reader.SetLayerMode(true);

  if (reader.ReadFile(filename) != IFSelect_RetDone || !reader.Transfer(doc)) {
    out.shape = readStepModel(filename);
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
    const Quantity_Color& fallbackColor) {
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
    const Quantity_Color& color) {
  if (shape.IsNull()) {
    return Handle(AIS_ColoredShape)();
  }
  Handle(AIS_ColoredShape) ais = new AIS_ColoredShape(shape);
  return shadeDefaults(ais, color);
}
