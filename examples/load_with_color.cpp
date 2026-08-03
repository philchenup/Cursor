/**
 * 在原有导入逻辑上改为带颜色显示：
 *
 *   AIS_Shape* loadShape;
 *   TopoDS_Shape scene_shape;
 *   if (suffix == "stl") {
 *     scene_shape = ReadModel::readStlModel(...);
 *   } else if (suffix == "step") {
 *     scene_shape = ReadModel::readStepModel(...);
 *   }
 *   loadShape = new AIS_Shape(scene_shape);
 */

#include "readmodel.h"

#include <AIS_ColoredShape.hxx>
#include <AIS_InteractiveContext.hxx>
#include <Quantity_Color.hxx>

#include <QString>

// 建议成员：
//   Handle(AIS_InteractiveContext) context;
//   Handle(AIS_ColoredShape) loadShape;

void importModelExample(const QString& filename,
                        const QString& suffix,
                        const Handle(AIS_InteractiveContext)& context,
                        Handle(AIS_ColoredShape)& loadShape)
{
    const Quantity_Color defaultTint(0.72, 0.74, 0.78, Quantity_TOC_RGB);

    if (suffix.compare(QStringLiteral("stl"), Qt::CaseInsensitive) == 0) {
        TopoDS_Shape scene_shape =
            ReadModel::readStlModel(filename.toStdString().c_str());
        if (scene_shape.IsNull()) {
            return;
        }
        // STL 无 CAD 颜色，设单色着色显示
        loadShape = ReadModel::makeDisplayShape(scene_shape, defaultTint);
    }
    else if (suffix.compare(QStringLiteral("step"), Qt::CaseInsensitive) == 0 ||
             suffix.compare(QStringLiteral("stp"), Qt::CaseInsensitive) == 0) {
        // 用 XCAF 读 STEP，保留面/实体颜色
        ReadModel::ColoredModel colored =
            ReadModel::readStepModelWithColors(filename.toStdString().c_str());
        if (colored.shape.IsNull()) {
            return;
        }
        loadShape = ReadModel::makeDisplayShape(colored, defaultTint);
    }
    else {
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
