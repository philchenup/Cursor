#ifndef READMODEL_H
#define READMODEL_H

#include <TopoDS.hxx>
#include <StlAPI_Reader.hxx>
#include <StlAPI_Writer.hxx>
#include <AIS_Shape.hxx>
#include <AIS_ColoredShape.hxx>
#include <Quantity_Color.hxx>
#include <STEPControl_Reader.hxx>
#include <STEPControl_Writer.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <TDocStd_Document.hxx>
#include <Standard_Handle.hxx>
#include <gp_Trsf.hxx>
#include <gp_Pnt.hxx>

/// STEP 文件长度单位（仅区分用户关心的 m / mm）。
enum class StepLengthUnit {
    Metre,        ///< 米，读取后不换算
    Millimetre,   ///< 毫米，绕包围盒圆心缩放到米
    Other         ///< 其它单位，按检测到的米换算系数缩放到米
};

/// 以包围盒圆心 (x,y,z) 为缩放中心等比例缩放。
TopoDS_Shape ScaleShape(const TopoDS_Shape& inputShape, double scaleFactor);

/// 以指定圆心 (x,y,z) 为缩放中心等比例缩放。
TopoDS_Shape ScaleShape(const TopoDS_Shape& inputShape,
                        double scaleFactor,
                        const gp_Pnt& center);

class ReadModel
{
public:
    /// STEP 带颜色结果：shape 始终为几何（内部单位为米）；xcafDoc 非空时含 XCAF 颜色。
    struct ColoredModel {
        TopoDS_Shape shape;
        Handle(TDocStd_Document) xcafDoc;
        bool hasColors = false;
        StepLengthUnit sourceUnit = StepLengthUnit::Millimetre;
        bool convertedToMetres = false;
        gp_Trsf unitTrsf; ///< mm→m 时的变换；米文件为单位阵
    };

    ReadModel();

    static TopoDS_Shape readStlModel(Standard_CString filePath);

    /// 仅几何。mm 文件绕圆心缩放到米；m 文件原样返回。
    static TopoDS_Shape readStepModel(Standard_CString filePath);

    /// STEP + XCAF 颜色。单位规则与 readStepModel 相同。
    static ColoredModel readStepModelWithColors(Standard_CString filePath);

    /// 由 ColoredModel / 普通 Shape 生成可着色显示对象。
    static Handle(AIS_ColoredShape) makeDisplayShape(
        const ColoredModel& model,
        const Quantity_Color& fallbackColor = Quantity_Color(0.75, 0.75, 0.80, Quantity_TOC_RGB));

    static Handle(AIS_ColoredShape) makeDisplayShape(
        const TopoDS_Shape& shape,
        const Quantity_Color& color = Quantity_Color(0.75, 0.75, 0.80, Quantity_TOC_RGB));

    static void writeStepModel(TopoDS_Shape shape, Standard_CString filePath);

    static void writeStlModel(TopoDS_Shape shape, Standard_CString filePath);

    /// 从已 ReadFile 的 STEPControl_Reader 判断长度单位。
    static StepLengthUnit detectStepLengthUnit(const STEPControl_Reader& reader);
};

#endif // READMODEL_H
