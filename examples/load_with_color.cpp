/**
 * Drop-in pattern for the original import path:
 *
 *   AIS_Shape* loadShape;
 *   TopoDS_Shape scene_shape;
 *   if (suffix == "stl")  scene_shape = ReadModel::readStlModel(...);
 *   else if (suffix == "step") scene_shape = ReadModel::readStepModel(...);
 *   loadShape = new AIS_Shape(scene_shape);
 *
 * Updated so STEP file colors are rendered (and STL gets a solid tint).
 *
 * Requires: OCCT with TKXDESTEP / TKXCAF linked.
 */

#include "ReadModel.h"

#include <AIS_ColoredShape.hxx>
#include <AIS_InteractiveContext.hxx>
#include <Quantity_Color.hxx>

#include <QString>

// Illustrative members assumed from the host UI class:
//   Handle(AIS_InteractiveContext) context;
//   Handle(AIS_ColoredShape) loadShape;   // prefer Handle<> over raw AIS_Shape*
//   QPlainTextEdit / console with print(...)

void importModelExample(const QString& filename,
                        const QString& suffix,
                        const Handle(AIS_InteractiveContext)& context,
                        Handle(AIS_ColoredShape)& loadShape) {
  const Quantity_Color defaultTint(0.72, 0.74, 0.78, Quantity_TOC_RGB);

  if (suffix.compare("stl", Qt::CaseInsensitive) == 0) {
    TopoDS_Shape scene_shape =
        ReadModel::readStlModel(filename.toStdString().c_str());
    if (scene_shape.IsNull()) {
      // ui->console->print(ct::LOG_INFO, "STL 读取失败");
      return;
    }
    // STL 本身没有 CAD 颜色，给一个可渲染的实体色 + 着色显示。
    loadShape = ReadModel::makeDisplayShape(scene_shape, defaultTint);
  } else if (suffix.compare("step", Qt::CaseInsensitive) == 0 ||
             suffix.compare("stp", Qt::CaseInsensitive) == 0) {
    // 关键：用 XCAF 读 STEP，保留面/实体颜色。
    ReadModel::ColoredModel colored =
        ReadModel::readStepModelWithColors(filename.toStdString().c_str());
    if (colored.shape.IsNull()) {
      // ui->console->print(ct::LOG_INFO, "STEP 读取失败");
      return;
    }
    loadShape = ReadModel::makeDisplayShape(colored, defaultTint);
  } else {
    // ui->console->print(ct::LOG_INFO,
    //     QStringLiteral("导入文件格式非stl或step文件!"));
    return;
  }

  if (loadShape.IsNull() || context.IsNull()) {
    return;
  }

  context->Display(loadShape, Standard_False);
  context->SetDisplayMode(loadShape, AIS_Shaded, Standard_False);
  context->UpdateCurrentViewer();
}

/*
 * Minimal patch if you must keep AIS_Shape* loadShape:
 *
 *   // after creating AIS_Shape:
 *   loadShape = new AIS_Shape(scene_shape);
 *   loadShape->SetColor(Quantity_Color(0.72, 0.74, 0.78, Quantity_TOC_RGB));
 *   loadShape->SetDisplayMode(AIS_Shaded);
 *   loadShape->Attributes()->SetFaceBoundaryDraw(Standard_True);
 *
 * Note: that only sets ONE color. It will NOT show multi-color STEP paints.
 * For STEP colors you need STEPCAFControl_Reader + AIS_ColoredShape (above).
 */
