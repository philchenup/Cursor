#include "WeldCoordinateSystem.hxx"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepLProp_SLProps.hxx>
#include <BRep_Tool.hxx>
#include <Geom2d_Curve.hxx>
#include <Precision.hxx>
#include <TopExp.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopTools_ListIteratorOfListOfShape.hxx>
#include <TopTools_ListOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

namespace {

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

  // BRepAdaptor_Curve follows TopoDS_Edge orientation (+Y = edge sense).
  origin = p;
  yDir   = gp_Dir(d1);
  return Standard_True;
}

//! Outward geometric normal of @p face at the mid-point of @p edge on that face.
Standard_Boolean FaceNormalAtEdge(const TopoDS_Face& face,
                                  const TopoDS_Edge& edge,
                                  gp_Dir&            normal)
{
  Standard_Real first = 0.0;
  Standard_Real last  = 0.0;
  Handle(Geom2d_Curve) pcurve = BRep_Tool::CurveOnSurface(edge, face, first, last);

  // Edge orientation in the caller may differ from the face wire; try reverse.
  if (pcurve.IsNull()) {
    const TopoDS_Edge rev = TopoDS::Edge(edge.Reversed());
    pcurve = BRep_Tool::CurveOnSurface(rev, face, first, last);
  }

  if (pcurve.IsNull() || Abs(last - first) <= Precision::PConfusion()) {
    return Standard_False;
  }

  const Standard_Real uMid = 0.5 * (first + last);
  const gp_Pnt2d      uv   = pcurve->Value(uMid);

  BRepAdaptor_Surface surf(face, Standard_True);
  BRepLProp_SLProps   props(surf, uv.X(), uv.Y(), 1, Precision::Confusion());
  if (!props.IsNormalDefined()) {
    return Standard_False;
  }

  normal = props.Normal();

  // For a solid shell, FORWARD faces have outward normals; REVERSED flips.
  if (face.Orientation() == TopAbs_REVERSED) {
    normal.Reverse();
  }
  return Standard_True;
}

//! Faces of @p shape that contain @p edge (IsSame match).
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

  // Bisector of the two face normals = dihedral open-angle direction → +Z.
  gp_Vec bisector(gp_Vec(n1.XYZ()) + gp_Vec(n2.XYZ()));
  if (bisector.Magnitude() <= Precision::Confusion()) {
    // Nearly coplanar opposite normals; fall back to either face normal.
    bisector = gp_Vec(n1.XYZ());
  }

  // Keep Z orthogonal to the edge tangent so the triad stays orthonormal.
  gp_Vec zVec = bisector - bisector.Dot(gp_Vec(yDir)) * gp_Vec(yDir);
  if (zVec.Magnitude() <= Precision::Confusion()) {
    return Standard_False;
  }
  const gp_Dir zDir(zVec);

  // Right-hand rule: X = Y × Z  (then X × Y = Z).
  gp_Vec xVec = gp_Vec(yDir).Crossed(gp_Vec(zDir));
  if (xVec.Magnitude() <= Precision::Confusion()) {
    return Standard_False;
  }
  gp_Dir xDir(xVec);

  // gp_Ax3(P, N=Z, Vx=X) builds Y as Z × X, which matches +edge when X = Y × Z.
  weldAxis = gp_Ax3(origin, zDir, xDir);
  if (weldAxis.YDirection().Dot(yDir) < 0.0) {
    xDir.Reverse();
    weldAxis = gp_Ax3(origin, zDir, xDir);
  }

  return Standard_True;
}
