#ifndef READMODEL_H
#define READMODEL_H

#include <AIS_ColoredShape.hxx>
#include <AIS_Shape.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <Quantity_Color.hxx>
#include <Standard_Handle.hxx>
#include <Standard_Transient.hxx>
#include <Standard_Type.hxx>
#include <STEPControl_Reader.hxx>
#include <STEPControl_Writer.hxx>
#include <StlAPI_Reader.hxx>
#include <StlAPI_Writer.hxx>
#include <TDocStd_Document.hxx>
#include <TopoDS.hxx>

enum class LengthUnit {
    Unknown,
    Millimetre,
    Centimetre,
    Metre,
    Inch
};

/// 挂到 AIS_Shape::SetOwner，便于之后从显示对象取回单位。
class ModelUnitAttr : public Standard_Transient
{
    DEFINE_STANDARD_RTTI_INLINE(ModelUnitAttr, Standard_Transient)
public:
    LengthUnit unit = LengthUnit::Unknown;
    LengthUnit sourceUnit = LengthUnit::Unknown;
    ModelUnitAttr() = default;
    ModelUnitAttr(LengthUnit current, LengthUnit source)
        : unit(current), sourceUnit(source) {}
};

class ReadModel
{
public:
    using LengthUnit = ::LengthUnit;
    using UnitAttr = ModelUnitAttr;

    struct LoadedModel {
        TopoDS_Shape shape;
        LengthUnit unit = LengthUnit::Unknown;       ///< 当前几何坐标单位
        LengthUnit sourceUnit = LengthUnit::Unknown; ///< 文件中的原始单位
        Handle(TDocStd_Document) xcafDoc;
        bool hasColors = false;
    };

    using ColoredModel = LoadedModel;

    ReadModel();

    static const char* unitName(LengthUnit unit);
    static double toMetres(LengthUnit unit); ///< 1 个该单位等于多少米；Unknown 为 0
    static LengthUnit unitOf(const AIS_Shape* ais);
    static LengthUnit sourceUnitOf(const AIS_Shape* ais);

    /// 以原点为中心等比缩放，默认缩小 1000 倍（mm → m）。
    static TopoDS_Shape ScaleShape(const TopoDS_Shape& shape, double factor = 0.001);

    /// 以原点为中心缩小 AIS_Shape 显示（默认 ×0.001），不改底层 BRep。
    static void ScaleAisShape(AIS_Shape* ais, double factor = 0.001);

    /// 按扩展名读取 STL / STEP，记录单位，不自动缩放。
    static LoadedModel loadModel(Standard_CString filePath);
    static LoadedModel loadStlModel(Standard_CString filePath);
    static LoadedModel loadStepModel(Standard_CString filePath);
    static LoadedModel loadStepModelWithColors(Standard_CString filePath);

    static TopoDS_Shape readStlModel(Standard_CString filePath);

    /// 仅几何。毫米会绕原点缩放到米，返回的 shape 按米计。
    static TopoDS_Shape readStepModel(Standard_CString filePath);

    /// STEP + XCAF 颜色。毫米会缩放到米，unit 为 Metre，sourceUnit 为文件单位。
    static ColoredModel readStepModelWithColors(Standard_CString filePath);

    static Handle(AIS_ColoredShape) makeDisplayShape(
        const ColoredModel& model,
        const Quantity_Color& fallbackColor = Quantity_Color(0.75, 0.75, 0.80, Quantity_TOC_RGB));

    static Handle(AIS_ColoredShape) makeDisplayShape(
        const TopoDS_Shape& shape,
        const Quantity_Color& color = Quantity_Color(0.75, 0.75, 0.80, Quantity_TOC_RGB));

    static void writeStepModel(TopoDS_Shape shape, Standard_CString filePath);

    static void writeStlModel(TopoDS_Shape shape, Standard_CString filePath);

};

#endif // READMODEL_H
