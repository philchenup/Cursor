#include "readmodel.h"

#include <cctype>
#include <cmath>
#include <fstream>
#include <string>

#include <Aspect_TypeOfLine.hxx>
#include <BRep_Builder.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <Interface_Static.hxx>
#include <OSD_Path.hxx>
#include <Poly_Triangulation.hxx>
#include <Prs3d_Drawer.hxx>
#include <Prs3d_LineAspect.hxx>
#include <RWStl.hxx>
#include <StepBasic_ConversionBasedUnitAndLengthUnit.hxx>
#include <StepBasic_SiPrefix.hxx>
#include <StepBasic_SiUnitAndLengthUnit.hxx>
#include <StepBasic_SiUnitName.hxx>
#include <STEPCAFControl_Reader.hxx>
#include <StepData_StepModel.hxx>
#include <TCollection_AsciiString.hxx>
#include <TDF_Label.hxx>
#include <TDF_LabelSequence.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Face.hxx>
#include <XCAFApp_Application.hxx>
#include <XCAFDoc_ColorTool.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>

using LengthUnit = ReadModel::LengthUnit;

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

std::string toUpper(std::string s)
{
    for (char& c : s) {
        c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    }
    return s;
}

LengthUnit parseUnitText(const std::string& text)
{
    const std::string s = toUpper(text);
    if (s.find("MILLI") != std::string::npos
        || s.find("UNIT=MM") != std::string::npos
        || s.find("UNIT:MM") != std::string::npos
        || s == "MM") {
        return LengthUnit::Millimetre;
    }
    if (s.find("CENTI") != std::string::npos || s == "CM") {
        return LengthUnit::Centimetre;
    }
    if (s.find("INCH") != std::string::npos || s == "IN") {
        return LengthUnit::Inch;
    }
    if (s.find("METRE") != std::string::npos
        || s.find("METER") != std::string::npos
        || s.find("UNIT=M") != std::string::npos
        || s.find("UNIT:M") != std::string::npos
        || s == "M") {
        return LengthUnit::Metre;
    }
    return LengthUnit::Unknown;
}

LengthUnit detectStlUnit(const char* path)
{
    if (!path) {
        return LengthUnit::Unknown;
    }
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        return LengthUnit::Unknown;
    }
    char buf[128]{};
    in.read(buf, 80);
    const auto n = static_cast<std::size_t>(in.gcount());
    if (n == 0) {
        return LengthUnit::Unknown;
    }
    return parseUnitText(std::string(buf, n));
}

LengthUnit detectStepUnit(const Handle(StepData_StepModel)& model)
{
    if (model.IsNull()) {
        return LengthUnit::Unknown;
    }

    LengthUnit named = LengthUnit::Unknown;
    bool sawMetre = false;
    bool sawMilli = false;
    bool sawCenti = false;

    for (Standard_Integer i = 1; i <= model->NbEntities(); ++i) {
        Handle(StepBasic_ConversionBasedUnitAndLengthUnit) conv =
            Handle(StepBasic_ConversionBasedUnitAndLengthUnit)::DownCast(model->Value(i));
        if (!conv.IsNull() && !conv->Name().IsNull()) {
            const LengthUnit u = parseUnitText(conv->Name()->ToCString());
            if (u != LengthUnit::Unknown) {
                named = u;
                if (u == LengthUnit::Millimetre || u == LengthUnit::Inch) {
                    return u;
                }
            }
        }

        Handle(StepBasic_SiUnitAndLengthUnit) si =
            Handle(StepBasic_SiUnitAndLengthUnit)::DownCast(model->Value(i));
        if (!si.IsNull() && si->Name() == StepBasic_sunMetre) {
            if (si->HasPrefix() && si->Prefix() == StepBasic_spMilli) {
                sawMilli = true;
            } else if (si->HasPrefix() && si->Prefix() == StepBasic_spCenti) {
                sawCenti = true;
            } else if (!si->HasPrefix()) {
                sawMetre = true;
            }
        }
    }

    if (named != LengthUnit::Unknown) {
        return named;
    }
    if (sawMilli) {
        return LengthUnit::Millimetre;
    }
    if (sawCenti) {
        return LengthUnit::Centimetre;
    }
    if (sawMetre) {
        return LengthUnit::Metre;
    }
    return LengthUnit::Unknown;
}

LengthUnit detectXcafUnit(const Handle(TDocStd_Document)& doc)
{
    if (doc.IsNull()) {
        return LengthUnit::Unknown;
    }
    Standard_Real metres = 0.0;
    if (!XCAFDoc_DocumentTool::GetLengthUnit(doc, metres)) {
        return LengthUnit::Unknown;
    }
    if (std::abs(metres - 0.001) < 1e-9) {
        return LengthUnit::Millimetre;
    }
    if (std::abs(metres - 0.01) < 1e-9) {
        return LengthUnit::Centimetre;
    }
    if (std::abs(metres - 0.0254) < 1e-6) {
        return LengthUnit::Inch;
    }
    if (std::abs(metres - 1.0) < 1e-6) {
        return LengthUnit::Metre;
    }
    return LengthUnit::Unknown;
}

void recordXcafUnit(const Handle(TDocStd_Document)& doc, LengthUnit unit)
{
    const double metres = ReadModel::toMetres(unit);
    if (doc.IsNull() || metres <= 0.0) {
        return;
    }
    XCAFDoc_DocumentTool::SetLengthUnit(doc, metres);
}

LengthUnit resolveStepUnit(const Handle(StepData_StepModel)& model,
    const Handle(TDocStd_Document)& doc = Handle(TDocStd_Document)())
{
    LengthUnit u = detectStepUnit(model);
    if (u == LengthUnit::Unknown) {
        u = detectXcafUnit(doc);
    }
    // STEP 未写单位时按常见 CAD 约定记为毫米
    return u == LengthUnit::Unknown ? LengthUnit::Millimetre : u;
}

const char* cascadeUnitName(LengthUnit unit)
{
    switch (unit) {
    case LengthUnit::Metre: return "M";
    case LengthUnit::Centimetre: return "CM";
    case LengthUnit::Inch: return "INCH";
    default: return "MM";
    }
}

TopoDS_Shape makeStlShape(Standard_CString filePath)
{
    Handle(Poly_Triangulation) tri = RWStl::ReadFile(OSD_Path(filePath));
    if (tri.IsNull()) {
        return {};
    }
    TopoDS_Face face;
    BRep_Builder builder;
    builder.MakeFace(face, tri);
    return face;
}

TopoDS_Shape takeFreeShapes(const Handle(TDocStd_Document)& doc)
{
    Handle(XCAFDoc_ShapeTool) shapeTool = XCAFDoc_DocumentTool::ShapeTool(doc->Main());
    TDF_LabelSequence freeShapes;
    shapeTool->GetFreeShapes(freeShapes);
    if (freeShapes.Length() == 0) {
        return {};
    }
    if (freeShapes.Length() == 1) {
        return shapeTool->GetShape(freeShapes.Value(1));
    }
    TopoDS_Compound compound;
    BRep_Builder builder;
    builder.MakeCompound(compound);
    for (Standard_Integer i = 1; i <= freeShapes.Length(); ++i) {
        builder.Add(compound, shapeTool->GetShape(freeShapes.Value(i)));
    }
    return compound;
}

ReadModel::LoadedModel convertToMetres(ReadModel::LoadedModel model)
{
    const double f = ReadModel::toMetres(model.unit);
    if (f > 0.0 && std::abs(f - 1.0) > 1e-12 && !model.shape.IsNull()) {
        model.shape = ReadModel::ScaleShape(model.shape, f);
        model.unit = LengthUnit::Metre;
    }
    return model;
}

bool endsWith(const TCollection_AsciiString& s, const char* suffix)
{
    const TCollection_AsciiString suf(suffix);
    if (s.Length() < suf.Length()) {
        return false;
    }
    return s.SubString(s.Length() - suf.Length() + 1, s.Length()).IsEqual(suf);
}

bool isStlPath(const TCollection_AsciiString& lower)
{
    return endsWith(lower, ".stl");
}

bool isStepPath(const TCollection_AsciiString& lower)
{
    return endsWith(lower, ".step") || endsWith(lower, ".stp") || endsWith(lower, ".p21")
        || endsWith(lower, ".ste");
}

} // namespace

ReadModel::ReadModel()
{
}

const char* ReadModel::unitName(LengthUnit unit)
{
    switch (unit) {
    case LengthUnit::Millimetre: return "mm";
    case LengthUnit::Centimetre: return "cm";
    case LengthUnit::Metre: return "m";
    case LengthUnit::Inch: return "in";
    default: return "unknown";
    }
}

double ReadModel::toMetres(LengthUnit unit)
{
    switch (unit) {
    case LengthUnit::Millimetre: return 0.001;
    case LengthUnit::Centimetre: return 0.01;
    case LengthUnit::Metre: return 1.0;
    case LengthUnit::Inch: return 0.0254;
    default: return 0.0;
    }
}

LengthUnit ReadModel::unitOf(const AIS_Shape* ais)
{
    if (!ais) {
        return LengthUnit::Unknown;
    }
    Handle(ModelUnitAttr) attr = Handle(ModelUnitAttr)::DownCast(ais->GetOwner());
    return attr.IsNull() ? LengthUnit::Unknown : attr->unit;
}

LengthUnit ReadModel::sourceUnitOf(const AIS_Shape* ais)
{
    if (!ais) {
        return LengthUnit::Unknown;
    }
    Handle(ModelUnitAttr) attr = Handle(ModelUnitAttr)::DownCast(ais->GetOwner());
    return attr.IsNull() ? LengthUnit::Unknown : attr->sourceUnit;
}

TopoDS_Shape ReadModel::ScaleShape(const TopoDS_Shape& shape, double factor)
{
    if (shape.IsNull()) {
        return {};
    }
    gp_Trsf t;
    t.SetScale(gp_Pnt(0, 0, 0), factor);
    return BRepBuilderAPI_Transform(shape, t, Standard_True).Shape();
}

void ReadModel::ScaleAisShape(AIS_Shape* ais, double factor)
{
    if (!ais) {
        return;
    }
    gp_Trsf t;
    t.SetScale(gp_Pnt(0, 0, 0), factor);
    ais->SetLocalTransformation(t);
    ais->Redisplay();
}

ReadModel::LoadedModel ReadModel::loadStlModel(Standard_CString filePath)
{
    LoadedModel out;
    if (!filePath) {
        return out;
    }
    out.sourceUnit = detectStlUnit(filePath);
    out.unit = out.sourceUnit;
    out.shape = makeStlShape(filePath);
    return out;
}

ReadModel::LoadedModel ReadModel::loadStepModel(Standard_CString filePath)
{
    LoadedModel out;
    if (!filePath) {
        return out;
    }

    STEPControl_Reader reader;
    if (reader.ReadFile(filePath) != IFSelect_RetDone) {
        return out;
    }

    out.sourceUnit = resolveStepUnit(reader.StepModel());
    out.unit = out.sourceUnit;
    Interface_Static::SetCVal("read.step.unit", cascadeUnitName(out.unit));
    reader.TransferRoots();
    out.shape = reader.OneShape();
    return out;
}

ReadModel::LoadedModel ReadModel::loadStepModelWithColors(Standard_CString filePath)
{
    LoadedModel out;
    if (!filePath) {
        return out;
    }

    Handle(TDocStd_Document) doc = newXcafDocument();
    STEPCAFControl_Reader reader;
    reader.SetColorMode(true);
    reader.SetNameMode(true);
    reader.SetLayerMode(true);

    if (reader.ReadFile(filePath) != IFSelect_RetDone) {
        return loadStepModel(filePath);
    }

    out.sourceUnit = resolveStepUnit(reader.ChangeReader().StepModel());
    Interface_Static::SetCVal("read.step.unit", cascadeUnitName(out.sourceUnit));
    if (!reader.Transfer(doc)) {
        return loadStepModel(filePath);
    }

    out.sourceUnit = resolveStepUnit(reader.ChangeReader().StepModel(), doc);
    out.unit = out.sourceUnit;
    recordXcafUnit(doc, out.unit);
    out.shape = takeFreeShapes(doc);
    out.xcafDoc = doc;

    Handle(XCAFDoc_ColorTool) colorTool = XCAFDoc_DocumentTool::ColorTool(doc->Main());
    TDF_LabelSequence colorLabels;
    colorTool->GetColors(colorLabels);
    out.hasColors = colorLabels.Length() > 0;
    return out;
}

ReadModel::LoadedModel ReadModel::loadModel(Standard_CString filePath)
{
    if (!filePath) {
        return {};
    }
    TCollection_AsciiString lower(filePath);
    lower.LowerCase();
    if (isStlPath(lower)) {
        return loadStlModel(filePath);
    }
    if (isStepPath(lower)) {
        return loadStepModelWithColors(filePath);
    }
    LoadedModel step = loadStepModelWithColors(filePath);
    if (!step.shape.IsNull()) {
        return step;
    }
    return loadStlModel(filePath);
}

TopoDS_Shape ReadModel::readStlModel(Standard_CString filePath)
{
    return loadStlModel(filePath).shape;
}

TopoDS_Shape ReadModel::readStepModel(Standard_CString filePath)
{
    return convertToMetres(loadStepModel(filePath)).shape;
}

ReadModel::ColoredModel ReadModel::readStepModelWithColors(Standard_CString filePath)
{
    return convertToMetres(loadStepModelWithColors(filePath));
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
    ais->SetOwner(new ModelUnitAttr(model.unit, model.sourceUnit));

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
    if (shape.IsNull()) {
        return;
    }
    Interface_Static::SetCVal("write.step.unit", "M");
    STEPControl_Writer write;
    write.Transfer(shape, STEPControl_AsIs);
    write.Write(filePath);
}

void ReadModel::writeStlModel(TopoDS_Shape shape, Standard_CString filePath)
{
    if (shape.IsNull()) {
        return;
    }
    StlAPI_Writer stlWriter;
    stlWriter.Write(shape, filePath);
}
