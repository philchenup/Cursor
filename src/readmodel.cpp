#include "readmodel.h"

#include <cmath>
#include <string>

#include <Interface_Static.hxx>
#include <RWStl.hxx>
#include <Poly_Triangulation.hxx>
#include <BRep_Builder.hxx>
#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Compound.hxx>
#include <OSD_Path.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <Aspect_TypeOfLine.hxx>
#include <Prs3d_Drawer.hxx>
#include <Prs3d_LineAspect.hxx>
#include <STEPCAFControl_Reader.hxx>
#include <Standard_Transient.hxx>
#include <StepData_StepModel.hxx>
#include <StepBasic_ConversionBasedUnit.hxx>
#include <StepBasic_ConversionBasedUnitAndLengthUnit.hxx>
#include <StepBasic_DimensionalExponents.hxx>
#include <StepBasic_HArray1OfNamedUnit.hxx>
#include <StepBasic_MeasureWithUnit.hxx>
#include <StepBasic_NamedUnit.hxx>
#include <StepBasic_SiPrefix.hxx>
#include <StepBasic_SiUnit.hxx>
#include <StepBasic_SiUnitAndLengthUnit.hxx>
#include <StepBasic_SiUnitName.hxx>
#include <StepBasic_Unit.hxx>
#include <StepGeom_GeometricRepresentationContextAndGlobalUnitAssignedContext.hxx>
#include <StepGeom_GeomRepContextAndGlobUnitAssCtxAndGlobUncertaintyAssCtx.hxx>
#include <StepRepr_GlobalUnitAssignedContext.hxx>
#include <TCollection_AsciiString.hxx>
#include <TCollection_HAsciiString.hxx>
#include <TDF_Label.hxx>
#include <TDF_LabelSequence.hxx>
#include <TopExp_Explorer.hxx>
#include <XCAFApp_Application.hxx>
#include <XCAFDoc_ColorTool.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <gp_Vec.hxx>

namespace {

constexpr double kMmToMetre = 0.001;
constexpr double kUnitEps = 1e-9;

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

class ScopedStaticCVal
{
public:
    ScopedStaticCVal(const char* key, const char* value)
        : key_(key)
    {
        const char* prev = Interface_Static::CVal(key);
        if (prev != nullptr) {
            prev_ = prev;
        }
        Interface_Static::SetCVal(key, value);
    }

    ~ScopedStaticCVal()
    {
        if (!prev_.empty()) {
            Interface_Static::SetCVal(key_, prev_.c_str());
        }
    }

private:
    const char* key_;
    std::string prev_;
};

double siPrefixToFactor(const StepBasic_SiPrefix prefix)
{
    switch (prefix) {
    case StepBasic_spExa:   return 1e18;
    case StepBasic_spPeta:  return 1e15;
    case StepBasic_spTera:  return 1e12;
    case StepBasic_spGiga:  return 1e9;
    case StepBasic_spMega:  return 1e6;
    case StepBasic_spKilo:  return 1e3;
    case StepBasic_spHecto: return 1e2;
    case StepBasic_spDeca:  return 1e1;
    case StepBasic_spDeci:  return 1e-1;
    case StepBasic_spCenti: return 1e-2;
    case StepBasic_spMilli: return 1e-3;
    case StepBasic_spMicro: return 1e-6;
    case StepBasic_spNano:  return 1e-9;
    case StepBasic_spPico:  return 1e-12;
    case StepBasic_spFemto: return 1e-15;
    case StepBasic_spAtto:  return 1e-18;
    default:                return 1.0;
    }
}

bool isLengthNamedUnit(const Handle(StepBasic_NamedUnit)& named)
{
    if (named.IsNull()) {
        return false;
    }
    if (!Handle(StepBasic_SiUnitAndLengthUnit)::DownCast(named).IsNull()) {
        return true;
    }
    if (!Handle(StepBasic_ConversionBasedUnitAndLengthUnit)::DownCast(named).IsNull()) {
        return true;
    }
    const Handle(StepBasic_DimensionalExponents) dim = named->Dimensions();
    if (dim.IsNull()) {
        return false;
    }
    return std::abs(dim->LengthExponent() - 1.0) < kUnitEps
        && std::abs(dim->MassExponent()) < kUnitEps
        && std::abs(dim->TimeExponent()) < kUnitEps
        && std::abs(dim->ElectricCurrentExponent()) < kUnitEps
        && std::abs(dim->ThermodynamicTemperatureExponent()) < kUnitEps
        && std::abs(dim->AmountOfSubstanceExponent()) < kUnitEps
        && std::abs(dim->LuminousIntensityExponent()) < kUnitEps;
}

bool namedUnitToMetres(const Handle(StepBasic_NamedUnit)& named,
                       double& metresPerUnit,
                       int depth = 0);

bool measureToMetres(const Handle(StepBasic_MeasureWithUnit)& measure,
                     double& metresPerUnit,
                     int depth)
{
    if (measure.IsNull()) {
        return false;
    }
    const double value = measure->ValueComponent();
    const StepBasic_Unit unit = measure->UnitComponent();
    const Handle(StepBasic_NamedUnit) ref = unit.NamedUnit();
    double refMetres = 1.0;
    if (!ref.IsNull() && namedUnitToMetres(ref, refMetres, depth + 1)) {
        metresPerUnit = value * refMetres;
        return true;
    }
    metresPerUnit = value;
    return true;
}

bool conversionBasedToMetres(const Handle(TCollection_HAsciiString)& convName,
                             const Handle(Standard_Transient)& factorEnt,
                             double& metresPerUnit,
                             int depth)
{
    if (!convName.IsNull()) {
        TCollection_AsciiString name(convName->ToCString());
        name.UpperCase();
        if (name.Search("MILLI") >= 1 || name.IsEqual("MM")) {
            metresPerUnit = kMmToMetre;
            return true;
        }
        if (name.IsEqual("METRE") || name.IsEqual("METER") || name.IsEqual("M")) {
            metresPerUnit = 1.0;
            return true;
        }
    }
    return measureToMetres(Handle(StepBasic_MeasureWithUnit)::DownCast(factorEnt),
                           metresPerUnit,
                           depth);
}

bool namedUnitToMetres(const Handle(StepBasic_NamedUnit)& named,
                       double& metresPerUnit,
                       int depth)
{
    if (named.IsNull() || depth > 8) {
        return false;
    }

    {
        const Handle(StepBasic_SiUnitAndLengthUnit) siLen =
            Handle(StepBasic_SiUnitAndLengthUnit)::DownCast(named);
        if (!siLen.IsNull() && siLen->Name() == StepBasic_sunMetre) {
            metresPerUnit = siLen->HasPrefix() ? siPrefixToFactor(siLen->Prefix()) : 1.0;
            return true;
        }
    }
    {
        const Handle(StepBasic_SiUnit) si = Handle(StepBasic_SiUnit)::DownCast(named);
        if (!si.IsNull() && si->Name() == StepBasic_sunMetre) {
            metresPerUnit = si->HasPrefix() ? siPrefixToFactor(si->Prefix()) : 1.0;
            return true;
        }
    }
    {
        const Handle(StepBasic_ConversionBasedUnitAndLengthUnit) convLen =
            Handle(StepBasic_ConversionBasedUnitAndLengthUnit)::DownCast(named);
        if (!convLen.IsNull()) {
            return conversionBasedToMetres(convLen->Name(),
                                           convLen->ConversionFactor(),
                                           metresPerUnit,
                                           depth);
        }
    }
    {
        const Handle(StepBasic_ConversionBasedUnit) conv =
            Handle(StepBasic_ConversionBasedUnit)::DownCast(named);
        if (!conv.IsNull()) {
            return conversionBasedToMetres(conv->Name(),
                                           conv->ConversionFactor(),
                                           metresPerUnit,
                                           depth);
        }
    }

    return false;
}

Handle(StepBasic_HArray1OfNamedUnit) unitsOfContext(const Handle(Standard_Transient)& ent)
{
    {
        const Handle(StepGeom_GeomRepContextAndGlobUnitAssCtxAndGlobUncertaintyAssCtx) ctx =
            Handle(StepGeom_GeomRepContextAndGlobUnitAssCtxAndGlobUncertaintyAssCtx)::DownCast(ent);
        if (!ctx.IsNull()) {
            return ctx->Units();
        }
    }
    {
        const Handle(StepGeom_GeometricRepresentationContextAndGlobalUnitAssignedContext) ctx =
            Handle(StepGeom_GeometricRepresentationContextAndGlobalUnitAssignedContext)::DownCast(ent);
        if (!ctx.IsNull()) {
            return ctx->Units();
        }
    }
    {
        const Handle(StepRepr_GlobalUnitAssignedContext) ctx =
            Handle(StepRepr_GlobalUnitAssignedContext)::DownCast(ent);
        if (!ctx.IsNull()) {
            return ctx->Units();
        }
    }
    return Handle(StepBasic_HArray1OfNamedUnit)();
}

struct DetectedLengthUnit {
    StepLengthUnit unit = StepLengthUnit::Millimetre;
    double metresPerFileUnit = kMmToMetre;
    const char* readStepUnit = "MM";
};

StepLengthUnit classifyMetresPerUnit(double metresPerFileUnit)
{
    if (std::abs(metresPerFileUnit - 1.0) < kUnitEps) {
        return StepLengthUnit::Metre;
    }
    if (std::abs(metresPerFileUnit - kMmToMetre) < 1e-12) {
        return StepLengthUnit::Millimetre;
    }
    return StepLengthUnit::Other;
}

const char* occtUnitName(double metresPerFileUnit)
{
    if (std::abs(metresPerFileUnit - 1.0) < kUnitEps) {
        return "M";
    }
    if (std::abs(metresPerFileUnit - kMmToMetre) < 1e-12) {
        return "MM";
    }
    if (std::abs(metresPerFileUnit - 0.01) < 1e-12) {
        return "CM";
    }
    if (std::abs(metresPerFileUnit - 1e-6) < 1e-15) {
        return "UM";
    }
    if (std::abs(metresPerFileUnit - 0.0254) < 1e-9) {
        return "INCH";
    }
    if (std::abs(metresPerFileUnit - 0.3048) < 1e-9) {
        return "FT";
    }
    return "MM";
}

bool takeLengthUnit(const Handle(StepBasic_NamedUnit)& named, DetectedLengthUnit& out)
{
    if (!isLengthNamedUnit(named)) {
        return false;
    }
    double metresPerUnit = kMmToMetre;
    if (!namedUnitToMetres(named, metresPerUnit)) {
        return false;
    }
    out.metresPerFileUnit = metresPerUnit;
    out.unit = classifyMetresPerUnit(metresPerUnit);
    out.readStepUnit = occtUnitName(metresPerUnit);
    return true;
}

DetectedLengthUnit detectLengthUnit(const Handle(StepData_StepModel)& model)
{
    DetectedLengthUnit detected;
    if (model.IsNull()) {
        return detected;
    }

    const Standard_Integer nb = model->NbEntities();
    for (Standard_Integer i = 1; i <= nb; ++i) {
        const Handle(StepBasic_HArray1OfNamedUnit) units = unitsOfContext(model->Value(i));
        if (units.IsNull()) {
            continue;
        }
        for (Standard_Integer u = units->Lower(); u <= units->Upper(); ++u) {
            if (takeLengthUnit(units->Value(u), detected)) {
                return detected;
            }
        }
    }

    for (Standard_Integer i = 1; i <= nb; ++i) {
        const Handle(StepBasic_NamedUnit) named =
            Handle(StepBasic_NamedUnit)::DownCast(model->Value(i));
        if (takeLengthUnit(named, detected)) {
            return detected;
        }
    }

    return detected;
}

bool bboxCenter(const TopoDS_Shape& shape, gp_Pnt& center)
{
    Bnd_Box box;
    BRepBndLib::Add(shape, box);
    if (box.IsVoid()) {
        center = gp_Pnt(0.0, 0.0, 0.0);
        return false;
    }
    Standard_Real xmin = 0.0;
    Standard_Real ymin = 0.0;
    Standard_Real zmin = 0.0;
    Standard_Real xmax = 0.0;
    Standard_Real ymax = 0.0;
    Standard_Real zmax = 0.0;
    box.Get(xmin, ymin, zmin, xmax, ymax, zmax);
    center = gp_Pnt(0.5 * (xmin + xmax),
                    0.5 * (ymin + ymax),
                    0.5 * (zmin + zmax));
    return true;
}

/// 绕包围盒圆心 (x,y,z) 缩放，并把圆心坐标一并换算到目标单位（世界坐标正确）。
gp_Trsf scaleAboutCenterToMetres(const TopoDS_Shape& shape, double metresPerFileUnit)
{
    gp_Pnt center(0.0, 0.0, 0.0);
    bboxCenter(shape, center);

    gp_Trsf scaleAboutCenter;
    scaleAboutCenter.SetScale(center, metresPerFileUnit);

    const gp_Pnt centerInMetres(center.X() * metresPerFileUnit,
                                center.Y() * metresPerFileUnit,
                                center.Z() * metresPerFileUnit);
    gp_Trsf moveCenter;
    moveCenter.SetTranslation(gp_Vec(center, centerInMetres));
    return moveCenter.Multiplied(scaleAboutCenter);
}

TopoDS_Shape applyFileUnitToMetres(const TopoDS_Shape& shape,
                                   const DetectedLengthUnit& unit,
                                   gp_Trsf& trsf,
                                   bool& converted)
{
    converted = false;
    trsf = gp_Trsf();
    if (shape.IsNull()) {
        return shape;
    }
    // 米制文件：原样读取，不做变换
    if (unit.unit == StepLengthUnit::Metre) {
        return shape;
    }

    trsf = scaleAboutCenterToMetres(shape, unit.metresPerFileUnit);
    BRepBuilderAPI_Transform transformTool(shape, trsf, Standard_True);
    transformTool.Build();
    const TopoDS_Shape scaled = transformTool.Shape();
    if (scaled.IsNull()) {
        return shape;
    }
    converted = true;
    return scaled;
}

TopoDS_Shape collectFreeShapes(const Handle(XCAFDoc_ShapeTool)& shapeTool)
{
    TDF_LabelSequence freeShapes;
    shapeTool->GetFreeShapes(freeShapes);
    if (freeShapes.Length() == 0) {
        return TopoDS_Shape();
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

} // namespace

ReadModel::ReadModel()
{
}

TopoDS_Shape ScaleShape(const TopoDS_Shape& inputShape,
                        double scaleFactor,
                        const gp_Pnt& center)
{
    if (inputShape.IsNull()) {
        return TopoDS_Shape();
    }

    gp_Trsf scaleTransform;
    scaleTransform.SetScale(center, scaleFactor);

    BRepBuilderAPI_Transform transformTool(inputShape, scaleTransform, Standard_True);
    transformTool.Build();
    TopoDS_Shape scaledShape = transformTool.Shape();
    if (scaledShape.IsNull()) {
        return TopoDS_Shape();
    }
    return scaledShape;
}

TopoDS_Shape ScaleShape(const TopoDS_Shape& inputShape, double scaleFactor)
{
    if (inputShape.IsNull()) {
        return TopoDS_Shape();
    }
    gp_Pnt center(0.0, 0.0, 0.0);
    bboxCenter(inputShape, center);
    return ScaleShape(inputShape, scaleFactor, center);
}

TopoDS_Shape ReadModel::readStlModel(Standard_CString filePath)
{
    Handle(Poly_Triangulation) tri = RWStl::ReadFile(OSD_Path(filePath));

    TopoDS_Shape shape_Stl;
    if (tri.IsNull()) {
        return shape_Stl;
    }

    TopoDS_Face face;
    BRep_Builder builder;
    builder.MakeFace(face, tri);

    shape_Stl = face;
    return shape_Stl;
}

StepLengthUnit ReadModel::detectStepLengthUnit(const STEPControl_Reader& reader)
{
    return detectLengthUnit(reader.StepModel()).unit;
}

TopoDS_Shape ReadModel::readStepModel(Standard_CString filePath)
{
    STEPControl_Reader stepReader;
    const IFSelect_ReturnStatus readStatus = stepReader.ReadFile(filePath);
    if (readStatus != IFSelect_RetDone) {
        return TopoDS_Shape();
    }

    const DetectedLengthUnit unit = detectLengthUnit(stepReader.StepModel());
    // 按文件自身单位传输，避免 OCCT 在 Transfer 时把 m 先换成 mm
    const ScopedStaticCVal unitGuard("read.step.unit", unit.readStepUnit);

    stepReader.TransferRoots();
    TopoDS_Shape shape = stepReader.OneShape();

    gp_Trsf trsf;
    bool converted = false;
    return applyFileUnitToMetres(shape, unit, trsf, converted);
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

    if (reader.ReadFile(filePath) != IFSelect_RetDone) {
        out.shape = readStepModel(filePath);
        return out;
    }

    const DetectedLengthUnit unit = detectLengthUnit(reader.ChangeReader().StepModel());
    const ScopedStaticCVal unitGuard("read.step.unit", unit.readStepUnit);

    if (!reader.Transfer(doc)) {
        out.shape = readStepModel(filePath);
        return out;
    }

    Handle(XCAFDoc_ShapeTool) shapeTool =
        XCAFDoc_DocumentTool::ShapeTool(doc->Main());
    Handle(XCAFDoc_ColorTool) colorTool =
        XCAFDoc_DocumentTool::ColorTool(doc->Main());

    out.shape = collectFreeShapes(shapeTool);
    if (out.shape.IsNull()) {
        return out;
    }

    TDF_LabelSequence colorLabels;
    colorTool->GetColors(colorLabels);
    out.hasColors = colorLabels.Length() > 0;
    out.xcafDoc = doc;
    out.sourceUnit = unit.unit;
    out.shape = applyFileUnitToMetres(out.shape, unit, out.unitTrsf, out.convertedToMetres);
    return out;
}

Handle(AIS_ColoredShape) ReadModel::makeDisplayShape(
    const ColoredModel& model,
    const Quantity_Color& fallbackColor)
{
    if (model.shape.IsNull()) {
        return Handle(AIS_ColoredShape)();
    }

    TopoDS_Shape displayShape = model.shape;
    const bool colorOnOriginal =
        model.hasColors && !model.xcafDoc.IsNull() && model.convertedToMetres;
    if (colorOnOriginal) {
        Handle(XCAFDoc_ShapeTool) origTool =
            XCAFDoc_DocumentTool::ShapeTool(model.xcafDoc->Main());
        displayShape = collectFreeShapes(origTool);
        if (displayShape.IsNull()) {
            displayShape = model.shape;
        }
    }

    Handle(AIS_ColoredShape) ais = new AIS_ColoredShape(displayShape);
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

    if (colorOnOriginal && model.unitTrsf.Form() != gp_Identity) {
        ais->SetLocalTransformation(model.unitTrsf);
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
