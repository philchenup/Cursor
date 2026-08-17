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

class ReadModel
{
public:
    /// STEP 带颜色结果：shape 始终为几何；xcafDoc 非空时含 XCAF 颜色。
    struct ColoredModel {
        TopoDS_Shape shape;
        Handle(TDocStd_Document) xcafDoc;
        bool hasColors = false;
    };

    ReadModel();

    static TopoDS_Shape readStlModel(Standard_CString filePath);

    /// 仅几何（原有接口，无颜色）。
    static TopoDS_Shape readStepModel(Standard_CString filePath);

    /// STEP + XCAF 颜色，用于可渲染多色的场景。
    static ColoredModel readStepModelWithColors(Standard_CString filePath);

    /// 由 ColoredModel / 普通 Shape 生成可着色显示对象。
    static Handle(AIS_ColoredShape) makeDisplayShape(
        const ColoredModel& model,
        const Quantity_Color& fallbackColor = Quantity_Color(0.75, 0.75, 0.80, Quantity_TOC_RGB));

    static Handle(AIS_ColoredShape) makeDisplayShape(
        const TopoDS_Shape& shape,
        const Quantity_Color& color = Quantity_Color(0.75, 0.75, 0.80, Quantity_TOC_RGB));

    /// 将 AIS 模型绕包围盒圆心缩小 scaleFactor 倍，返回新的 AIS_Shape*（不改原对象）。
    /// 用法: loadShape = ReadModel::ScaleAis(loadColorShape.get(), 0.001);
    static AIS_Shape* ScaleAis(AIS_Shape* ais, double scaleFactor);

    static void writeStepModel(TopoDS_Shape shape, Standard_CString filePath);

    static void writeStlModel(TopoDS_Shape shape, Standard_CString filePath);

};

#endif // READMODEL_H
