#pragma once

#include <AIS_ColoredShape.hxx>
#include <Quantity_Color.hxx>
#include <Standard_Handle.hxx>
#include <TDocStd_Document.hxx>
#include <TopoDS_Shape.hxx>

#include <string>

/// Load STL / STEP models. STEP colors are preserved via XCAF.
class ReadModel {
public:
  struct ColoredModel {
    TopoDS_Shape shape;                 ///< Combined shape (always filled when ok)
    Handle(TDocStd_Document) xcafDoc;   ///< Non-null only for colored STEP
    bool hasColors = false;
  };

  /// Plain STL -> TopoDS_Shape (STL has no CAD color attributes).
  static TopoDS_Shape readStlModel(const char* filename);

  /// Plain STEP geometry only (no colors). Kept for callers that only need shape.
  static TopoDS_Shape readStepModel(const char* filename);

  /// STEP with XCAF colors / names. Prefer this when you want to render colors.
  static ColoredModel readStepModelWithColors(const char* filename);

  /// Build an AIS object that shows per-face / per-solid STEP colors when available.
  /// Falls back to a single AIS_ColoredShape tinted with @p fallbackColor.
  static Handle(AIS_ColoredShape) makeDisplayShape(
      const ColoredModel& model,
      const Quantity_Color& fallbackColor = Quantity_Color(0.75, 0.75, 0.80, Quantity_TOC_RGB));

  /// Convenience: wrap a bare TopoDS_Shape (e.g. STL) with a display color.
  static Handle(AIS_ColoredShape) makeDisplayShape(
      const TopoDS_Shape& shape,
      const Quantity_Color& color = Quantity_Color(0.75, 0.75, 0.80, Quantity_TOC_RGB));
};
