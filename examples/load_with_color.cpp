/**
 * 在原有导入逻辑上改为带颜色显示。
 *
 * Handle(AIS_ColoredShape) -> AIS_Shape* ：
 *   AIS_ColoredShape 继承 AIS_Shape，用 .get() 取裸指针即可：
 *     Handle(AIS_ColoredShape) h = ReadModel::makeDisplayShape(...);
 *     AIS_Shape* loadShape = h.get();
 *
 * 注意：Display 进 context 后由 context 持有引用计数；若成员只有裸指针，
 * 建议同时保留一份 Handle，避免局部 Handle 析构后对象被释放。
 */

#include "readmodel.h"

#include <AIS_ColoredShape.hxx>
#include <AIS_InteractiveContext.hxx>
#include <AIS_Shape.hxx>
#include <Quantity_Color.hxx>

#include <QString>

// 成员保持原样：
//   Handle(AIS_InteractiveContext) context;
//   AIS_Shape* loadShape;
// 可选（推荐）：Handle(AIS_ColoredShape) loadShapeHandle; // 延长寿命

void importModelExample(const QString& filename,
                        const QString& suffix,
                        const Handle(AIS_InteractiveContext)& context,
                        AIS_Shape*& loadShape,
                        Handle(AIS_ColoredShape)& loadShapeHandle)
{
    const Quantity_Color defaultTint(0.72, 0.74, 0.78, Quantity_TOC_RGB);

    if (suffix.compare(QStringLiteral("stl"), Qt::CaseInsensitive) == 0) {
        TopoDS_Shape scene_shape =
            ReadModel::readStlModel(filename.toStdString().c_str());
        if (scene_shape.IsNull()) {
            return;
        }
        loadShapeHandle = ReadModel::makeDisplayShape(scene_shape, defaultTint);
    }
    else if (suffix.compare(QStringLiteral("step"), Qt::CaseInsensitive) == 0 ||
             suffix.compare(QStringLiteral("stp"), Qt::CaseInsensitive) == 0) {
        ReadModel::ColoredModel colored =
            ReadModel::readStepModelWithColors(filename.toStdString().c_str());
        if (colored.shape.IsNull()) {
            return;
        }
        loadShapeHandle = ReadModel::makeDisplayShape(colored, defaultTint);
    }
    else {
        // ui->console->print(ct::LOG_INFO,
        //     QStringLiteral("导入文件格式非stl或step文件!"));
        return;
    }

    // Handle(AIS_ColoredShape) -> AIS_Shape*
    loadShape = loadShapeHandle.get();

    if (loadShape == nullptr || context.IsNull()) {
        return;
    }

    // Display 接受 Handle；从裸指针再包一层也可以：
    //   Handle(AIS_Shape) h(loadShape);
    context->Display(loadShapeHandle, Standard_False);
    context->SetDisplayMode(loadShapeHandle, AIS_Shaded, Standard_False);
    context->UpdateCurrentViewer();
}
