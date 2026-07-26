#include "WeldCoordinateSystem.h"

namespace {

    //! Mid-parameter point and unit tangent of @p edge, respecting edge orientation.
    Standard_Boolean EdgeFrameSeed(const TopoDS_Edge& edge,
        gp_Pnt& origin,
        gp_Dir& yDir)
    {
        if (edge.IsNull() || BRep_Tool::Degenerated(edge)) {
            return Standard_False;
        }

        BRepAdaptor_Curve curve(edge);
        const Standard_Real uFirst = curve.FirstParameter();
        const Standard_Real uLast = curve.LastParameter();
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
        yDir = gp_Dir(d1);
        return Standard_True;
    }

    //! Face normal at edge parameter @p uOnEdge (same domain as BRepAdaptor_Curve).
    Standard_Boolean FaceNormalAtEdgeParam(const TopoDS_Face& face,
        const TopoDS_Edge& edge,
        Standard_Real      uOnEdge,
        gp_Dir& normal)
    {
        Standard_Real first = 0.0;
        Standard_Real last = 0.0;
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
        gp_Dir& normal)
    {
        Standard_Real f = 0.0;
        Standard_Real l = 0.0;
        BRep_Tool::Range(edge, f, l);
        return FaceNormalAtEdgeParam(face, edge, 0.5 * (f + l), normal);
    }

    void CollectAdjacentFaces(const TopoDS_Shape& shape,
        const TopoDS_Edge& edge,
        TopTools_ListOfShape& faces)
    {
        faces.Clear();

        TopTools_IndexedDataMapOfShapeListOfShape edgeToFaces;
        TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, edgeToFaces);

        for (Standard_Integer i = 1; i <= edgeToFaces.Extent(); ++i) {
            if (edgeToFaces.FindKey(i).IsPartner(edge)) {
                faces = edgeToFaces.FindFromIndex(i);
                return;
            }
        }
    }

    //! Project @p v onto the plane perpendicular to unit direction @p axis.
    Standard_Boolean ProjectPerp(const gp_Vec& v, const gp_Dir& axis, gp_Vec& out)
    {
        const gp_Vec a(axis);
        out = v - v.Dot(a) * a;
        if (out.Magnitude() <= Precision::Confusion()) {
            return Standard_False;
        }
        out.Normalize();
        return Standard_True;
    }

    //! Unit dihedral bisector of the two face normals at parameter @p u, ⊥ @p edgeDir.
    //! Uses projected unit normals so Z lies exactly on the angle bisector in the
    //! plane perpendicular to the edge.
    Standard_Boolean BisectorAtParam(const TopoDS_Face& face1,
        const TopoDS_Face& face2,
        const TopoDS_Edge& edge,
        Standard_Real      u,
        const gp_Dir& edgeDir,
        gp_Dir& zDir)
    {
        gp_Dir n1;
        gp_Dir n2;
        if (!FaceNormalAtEdgeParam(face1, edge, u, n1)
            || !FaceNormalAtEdgeParam(face2, edge, u, n2)) {
            return Standard_False;
        }

        gp_Vec p1;
        gp_Vec p2;
        if (!ProjectPerp(gp_Vec(n1), edgeDir, p1) || !ProjectPerp(gp_Vec(n2), edgeDir, p2)) {
            return Standard_False;
        }

        gp_Vec bisector = p1 + p2;
        if (bisector.Magnitude() <= Precision::Confusion()) {
            // Faces nearly coplanar / opposite: fall back to either projected normal.
            bisector = p1;
        }

        if (bisector.Magnitude() <= Precision::Confusion()) {
            return Standard_False;
        }

        zDir = gp_Dir(bisector);
        return Standard_True;
    }

    //! Build one sample: X along edge, Z = ± face-normal bisector, Y = Z × X.
    Standard_Boolean BuildDiscretePointAt(const TopoDS_Face& face1,
        const TopoDS_Face& face2,
        const TopoDS_Edge& edge,
        const BRepAdaptor_Curve& curve,
        Standard_Real            u,
        Standard_Boolean         reverseZ,
        const gp_Dir* prevZ,
        DiscretePoint& sample)
    {
        gp_Pnt p;
        gp_Vec d1;
        curve.D1(u, p, d1);
        if (d1.Magnitude() <= Precision::Confusion()) {
            return Standard_False;
        }

        const gp_Dir xDir(d1); // travel / edge tangent → X

        gp_Dir zDir;
        if (!BisectorAtParam(face1, face2, edge, u, xDir, zDir)) {
            return Standard_False;
        }

        // reverseZ=True → opposite bisector (二分角反向).
        if (reverseZ) {
            zDir.Reverse();
        }

        // Keep Z continuous along the path (avoids 180° flips / fan artifacts).
        if (prevZ != nullptr && zDir.Dot(*prevZ) < 0.0) {
            zDir.Reverse();
        }

        // Right-hand rule with X = edge tangent: Y = Z × X  ⇒  X × Y = Z.
        gp_Vec yVec = gp_Vec(zDir).Crossed(gp_Vec(xDir));
        if (yVec.Magnitude() <= Precision::Confusion()) {
            return Standard_False;
        }

        sample.position = p;
        sample.xDir = xDir;
        sample.zDir = zDir;
        sample.yDir = gp_Dir(yVec);
        return Standard_True;
    }

    Standard_Boolean BuildFrameFromFaces(const TopoDS_Face& face1,
        const TopoDS_Face& face2,
        const TopoDS_Edge& edge,
        const gp_Pnt& origin,
        const gp_Dir& yDir,
        Standard_Boolean   reverseZ,
        gp_Ax3& weldAxis)
    {
        Standard_Real f = 0.0;
        Standard_Real l = 0.0;
        BRep_Tool::Range(edge, f, l);

        gp_Dir zDir;
        if (!BisectorAtParam(face1, face2, edge, 0.5 * (f + l), yDir, zDir)) {
            return Standard_False;
        }

        if (reverseZ) {
            zDir.Reverse();
        }

        gp_Vec xVec = gp_Vec(yDir).Crossed(gp_Vec(zDir));
        if (xVec.Magnitude() <= Precision::Confusion()) {
            return Standard_False;
        }
        gp_Dir xDir(xVec);

        // gp_Ax3(P, N=Z, Vx=X) builds Y as Z × X, matching +edge when X = Y × Z.
        weldAxis = gp_Ax3(origin, zDir, xDir);
        if (weldAxis.YDirection().Dot(yDir) < 0.0) {
            xDir.Reverse();
            weldAxis = gp_Ax3(origin, zDir, xDir);
        }
        return Standard_True;
    }
} // namespace

Standard_Boolean ComputeWeldCoordinateSystem(const TopoDS_Shape& selectShape,
    const TopoDS_Edge& edge,
    gp_Ax3& weldAxis,
    Standard_Boolean    reverseZ)
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

    return BuildFrameFromFaces(face1, face2, edge, origin, yDir, reverseZ, weldAxis);
}

Standard_Boolean DiscretizeWeldTrajectory(const TopoDS_Shape& selectShape,
    const TopoDS_Edge& edge,
    std::vector<DiscretePoint>& trajectory,
    Standard_Real               spacingMm,
    Standard_Boolean            reverseZ,
    Standard_Real               retractMm)
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

    // Arc-length sampling; always includes both endpoints.
    GCPnts_UniformAbscissa sampler(curve, spacingMm);
    if (!sampler.IsDone() || sampler.NbPoints() < 1) {
        return Standard_False;
    }

    trajectory.reserve(static_cast<std::size_t>(sampler.NbPoints()) + 2);
    const gp_Dir* prevZ = nullptr;
    gp_Dir        lastZ;

    for (Standard_Integer i = 1; i <= sampler.NbPoints(); ++i) {
        DiscretePoint sample;
        if (!BuildDiscretePointAt(face1,
            face2,
            edge,
            curve,
            sampler.Parameter(i),
            reverseZ,
            prevZ,
            sample)) {
            trajectory.clear();
            return Standard_False;
        }
        trajectory.push_back(sample);
        lastZ = sample.zDir;
        prevZ = &lastZ;
    }

    if (trajectory.empty()) {
        return Standard_False;
    }

    // 起弧点 / 收弧点: retract the first and last seam points along -Z.
    if (retractMm > Precision::Confusion()) {
        const DiscretePoint& seamFirst = trajectory.front();
        const DiscretePoint& seamLast = trajectory.back();

        DiscretePoint startArc = seamFirst; // 起弧点
        startArc.position =
            seamFirst.position.Translated(gp_Vec(seamFirst.zDir) * (-retractMm));

        DiscretePoint endArc = seamLast; // 收弧点
        endArc.position =
            seamLast.position.Translated(gp_Vec(seamLast.zDir) * (-retractMm));

        trajectory.insert(trajectory.begin(), startArc);
        trajectory.push_back(endArc);
    }

    return Standard_True;
}