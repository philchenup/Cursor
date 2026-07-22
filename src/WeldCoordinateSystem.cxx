#include "WeldCoordinateSystem.hxx"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepLProp_SLProps.hxx>
#include <BRep_Tool.hxx>
#include <GCPnts_AbscissaPoint.hxx>
#include <GCPnts_UniformAbscissa.hxx>
#include <Geom2d_Curve.hxx>
#include <Precision.hxx>
#include <TopExp.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopTools_ListIteratorOfListOfShape.hxx>
#include <TopTools_ListOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <gp_Vec.hxx>

#include <algorithm>
#include <cmath>

namespace {

const Standard_Real kInvSqrt2 = 0.7071067811865476; // √2/2

//! Mid-parameter point and unit tangent of @p edge, respecting edge orientation.
Standard_Boolean EdgeFrameSeed(const TopoDS_Edge& edge,
                               gp_Pnt&            origin,
                               gp_Dir&            yDir)
{
  if (edge.IsNull() || BRep_Tool::Degenerated(edge)) {
    return Standard_False;
  }

  BRepAdaptor_Curve curve(edge);
  const Standard_Real uFirst = curve.FirstParameter();
  const Standard_Real uLast  = curve.LastParameter();
  if (Abs(uLast - uFirst) <= Precision::PConfusion()) {
    return Standard_False;
  }

  const Standard_Real uMid = 0.5 * (uFirst + uLast);
  gp_Pnt p;
  gp_Vec d1;
  curve.D1(uMid, p, d1);

  if (d1.Magnitude() <= Precision::Confusion()) {
    return Standard_False;
  }

  origin = p;
  yDir   = gp_Dir(d1);
  return Standard_True;
}

//! Face normal at edge parameter @p uOnEdge (same domain as BRepAdaptor_Curve).
Standard_Boolean FaceNormalAtEdgeParam(const TopoDS_Face& face,
                                       const TopoDS_Edge& edge,
                                       Standard_Real      uOnEdge,
                                       gp_Dir&            normal)
{
  Standard_Real first = 0.0;
  Standard_Real last  = 0.0;
  Handle(Geom2d_Curve) pcurve = BRep_Tool::CurveOnSurface(edge, face, first, last);
  Standard_Boolean     reversedPCurve = Standard_False;

  if (pcurve.IsNull()) {
    const TopoDS_Edge rev = TopoDS::Edge(edge.Reversed());
    pcurve = BRep_Tool::CurveOnSurface(rev, face, first, last);
    reversedPCurve = Standard_True;
  }

  if (pcurve.IsNull() || Abs(last - first) <= Precision::PConfusion()) {
    return Standard_False;
  }

  // Map 3D-edge parameter onto the pcurve domain when the reversed edge was used.
  Standard_Real u = uOnEdge;
  if (reversedPCurve) {
    u = first + last - uOnEdge;
  }
  u = std::max(first, std::min(last, u));

  const gp_Pnt2d uv = pcurve->Value(u);

  BRepAdaptor_Surface surf(face, Standard_True);
  BRepLProp_SLProps   props(surf, uv.X(), uv.Y(), 1, Precision::Confusion());
  if (!props.IsNormalDefined()) {
    return Standard_False;
  }

  normal = props.Normal();
  if (face.Orientation() == TopAbs_REVERSED) {
    normal.Reverse();
  }
  return Standard_True;
}

Standard_Boolean FaceNormalAtEdge(const TopoDS_Face& face,
                                  const TopoDS_Edge& edge,
                                  gp_Dir&            normal)
{
  Standard_Real f = 0.0;
  Standard_Real l = 0.0;
  BRep_Tool::Range(edge, f, l);
  return FaceNormalAtEdgeParam(face, edge, 0.5 * (f + l), normal);
}

void CollectAdjacentFaces(const TopoDS_Shape&   shape,
                          const TopoDS_Edge&     edge,
                          TopTools_ListOfShape& faces)
{
  faces.Clear();

  TopTools_IndexedDataMapOfShapeListOfShape edgeToFaces;
  TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, edgeToFaces);

  for (Standard_Integer i = 1; i <= edgeToFaces.Extent(); ++i) {
    if (edgeToFaces.FindKey(i).IsSame(edge)) {
      faces = edgeToFaces.FindFromIndex(i);
      return;
    }
  }
}

//! Open-angle unit bisector of the two face normals at parameter @p u, ⊥ @p xDir.
Standard_Boolean BisectorAtParam(const TopoDS_Face& face1,
                                 const TopoDS_Face& face2,
                                 const TopoDS_Edge& edge,
                                 Standard_Real      u,
                                 const gp_Dir&      xDir,
                                 gp_Vec&            bisectorOut)
{
  gp_Dir n1;
  gp_Dir n2;
  if (!FaceNormalAtEdgeParam(face1, edge, u, n1)
      || !FaceNormalAtEdgeParam(face2, edge, u, n2)) {
    return Standard_False;
  }

  gp_Vec bisector(gp_Vec(n1.XYZ()) + gp_Vec(n2.XYZ()));
  if (bisector.Magnitude() <= Precision::Confusion()) {
    bisector = gp_Vec(n1.XYZ());
  }

  const gp_Vec x(xDir);
  bisector -= bisector.Dot(x) * x;
  if (bisector.Magnitude() <= Precision::Confusion()) {
    return Standard_False;
  }
  bisector.Normalize();
  bisectorOut = bisector;
  return Standard_True;
}

//! Build Z: ⊥ X, angle with world XOY = 45° (clamped if needed), pointing down.
//! If two solutions exist, pick the one closer to @p prefer (typically the bisector).
Standard_Boolean MakeZDir45DownToXOY(const gp_Dir& xDir,
                                     const gp_Vec& prefer,
                                     gp_Dir&       zDir)
{
  const gp_Vec x(xDir);
  const gp_Vec k(0.0, 0.0, 1.0); // world +Z, XOY = world horizontal plane

  const Standard_Real xDotK   = x.Dot(k);
  const Standard_Real maxAbs  = std::sqrt(std::max(0.0, 1.0 - xDotK * xDotK));
  if (maxAbs <= Precision::Confusion()) {
    // Travel is vertical: every ⊥X direction lies in XOY (0°). Fall back to prefer.
    gp_Vec z = prefer - prefer.Dot(x) * x;
    if (z.Magnitude() <= Precision::Confusion()) {
      return Standard_False;
    }
    zDir = gp_Dir(z);
    return Standard_True;
  }

  // Target: angle with XOY plane = 45° downward ⇒ z·k = -sin(45°).
  const Standard_Real targetDot = -std::min(kInvSqrt2, maxAbs);

  gp_Vec ref = (std::abs(xDotK) < 0.9) ? k : gp_Vec(1.0, 0.0, 0.0);
  gp_Vec e1  = x.Crossed(ref);
  if (e1.Magnitude() <= Precision::Confusion()) {
    return Standard_False;
  }
  e1.Normalize();
  gp_Vec e2 = x.Crossed(e1);
  e2.Normalize();

  // z = cosθ e1 + sinθ e2, with z·k = targetDot
  // a cosθ + b sinθ = targetDot, a=e1·k, b=e2·k
  const Standard_Real a = e1.Dot(k);
  const Standard_Real b = e2.Dot(k);
  const Standard_Real R = std::sqrt(a * a + b * b);
  if (R <= Precision::Confusion() || std::abs(targetDot) > R + Precision::Confusion()) {
    return Standard_False;
  }

  const Standard_Real phi   = std::atan2(b, a);
  const Standard_Real cosV  = std::max(-1.0, std::min(1.0, targetDot / R));
  const Standard_Real delta = std::acos(cosV);

  auto makeZ = [&](Standard_Real theta) -> gp_Vec {
    return gp_Vec(std::cos(theta) * e1.XYZ() + std::sin(theta) * e2.XYZ());
  };

  const gp_Vec z1 = makeZ(phi + delta);
  const gp_Vec z2 = makeZ(phi - delta);

  gp_Vec pref = prefer - prefer.Dot(x) * x;
  if (pref.Magnitude() > Precision::Confusion()) {
    pref.Normalize();
    zDir = (z1.Dot(pref) >= z2.Dot(pref)) ? gp_Dir(z1) : gp_Dir(z2);
  }
  else {
    zDir = gp_Dir(z1);
  }
  return Standard_True;
}

Standard_Boolean BuildDiscretePointAt(const TopoDS_Face& face1,
                                      const TopoDS_Face& face2,
                                      const TopoDS_Edge& edge,
                                      const BRepAdaptor_Curve& curve,
                                      Standard_Real      u,
                                      DiscretePoint&     sample)
{
  gp_Pnt p;
  gp_Vec d1;
  curve.D1(u, p, d1);
  if (d1.Magnitude() <= Precision::Confusion()) {
    return Standard_False;
  }

  const gp_Dir xDir(d1);

  gp_Vec preferBisector;
  if (!BisectorAtParam(face1, face2, edge, u, xDir, preferBisector)) {
    // No reliable bisector: prefer world down projected ⊥ X.
    preferBisector = gp_Vec(0.0, 0.0, -1.0);
    preferBisector -= preferBisector.Dot(gp_Vec(xDir)) * gp_Vec(xDir);
    if (preferBisector.Magnitude() <= Precision::Confusion()) {
      preferBisector = gp_Vec(1.0, 0.0, 0.0).Crossed(gp_Vec(xDir));
    }
  }

  gp_Dir zDir;
  if (!MakeZDir45DownToXOY(xDir, preferBisector, zDir)) {
    return Standard_False;
  }

  gp_Vec yVec = gp_Vec(zDir).Crossed(gp_Vec(xDir)); // Y = Z × X
  if (yVec.Magnitude() <= Precision::Confusion()) {
    return Standard_False;
  }

  sample.position = p;
  sample.xDir     = xDir;
  sample.zDir     = zDir;
  sample.yDir     = gp_Dir(yVec);
  return Standard_True;
}

} // namespace

Standard_Boolean ComputeWeldCoordinateSystem(const TopoDS_Shape& selectShape,
                                             const TopoDS_Edge&   edge,
                                             gp_Ax3&             weldAxis)
{
  if (selectShape.IsNull() || edge.IsNull()) {
    return Standard_False;
  }

  TopTools_ListOfShape faceList;
  CollectAdjacentFaces(selectShape, edge, faceList);
  if (faceList.Extent() != 2) {
    return Standard_False;
  }

  TopTools_ListIteratorOfListOfShape it(faceList);
  const TopoDS_Face face1 = TopoDS::Face(it.Value());
  it.Next();
  const TopoDS_Face face2 = TopoDS::Face(it.Value());

  gp_Pnt origin;
  gp_Dir yDir;
  if (!EdgeFrameSeed(edge, origin, yDir)) {
    return Standard_False;
  }

  gp_Dir n1;
  gp_Dir n2;
  if (!FaceNormalAtEdge(face1, edge, n1) || !FaceNormalAtEdge(face2, edge, n2)) {
    return Standard_False;
  }

  gp_Vec bisector(gp_Vec(n1.XYZ()) + gp_Vec(n2.XYZ()));
  if (bisector.Magnitude() <= Precision::Confusion()) {
    bisector = gp_Vec(n1.XYZ());
  }

  gp_Vec zVec = bisector - bisector.Dot(gp_Vec(yDir)) * gp_Vec(yDir);
  if (zVec.Magnitude() <= Precision::Confusion()) {
    return Standard_False;
  }
  const gp_Dir zDir(zVec);

  gp_Vec xVec = gp_Vec(yDir).Crossed(gp_Vec(zDir));
  if (xVec.Magnitude() <= Precision::Confusion()) {
    return Standard_False;
  }
  gp_Dir xDir(xVec);

  weldAxis = gp_Ax3(origin, zDir, xDir);
  if (weldAxis.YDirection().Dot(yDir) < 0.0) {
    xDir.Reverse();
    weldAxis = gp_Ax3(origin, zDir, xDir);
  }

  return Standard_True;
}

Standard_Boolean DiscretizeWeldTrajectory(const TopoDS_Shape&        selectShape,
                                          const TopoDS_Edge&          edge,
                                          std::vector<DiscretePoint>& trajectory,
                                          Standard_Real               spacingMm)
{
  trajectory.clear();

  if (selectShape.IsNull() || edge.IsNull() || spacingMm <= Precision::Confusion()) {
    return Standard_False;
  }
  if (BRep_Tool::Degenerated(edge)) {
    return Standard_False;
  }

  TopTools_ListOfShape faceList;
  CollectAdjacentFaces(selectShape, edge, faceList);
  if (faceList.Extent() != 2) {
    return Standard_False;
  }

  TopTools_ListIteratorOfListOfShape it(faceList);
  const TopoDS_Face face1 = TopoDS::Face(it.Value());
  it.Next();
  const TopoDS_Face face2 = TopoDS::Face(it.Value());

  BRepAdaptor_Curve curve(edge);
  const Standard_Real length =
      GCPnts_AbscissaPoint::Length(curve, curve.FirstParameter(), curve.LastParameter());
  if (length <= Precision::Confusion()) {
    return Standard_False;
  }

  // Arc-length sampling; always includes both endpoints. If length < spacing,
  // OCC still returns the two endpoints.
  GCPnts_UniformAbscissa sampler(curve, spacingMm);
  if (!sampler.IsDone() || sampler.NbPoints() < 1) {
    return Standard_False;
  }

  trajectory.reserve(static_cast<std::size_t>(sampler.NbPoints()));
  for (Standard_Integer i = 1; i <= sampler.NbPoints(); ++i) {
    DiscretePoint sample;
    if (!BuildDiscretePointAt(face1, face2, edge, curve, sampler.Parameter(i), sample)) {
      trajectory.clear();
      return Standard_False;
    }
    trajectory.push_back(sample);
  }

  return !trajectory.empty();
}
