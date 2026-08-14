// 模型几何单位为米时：接口仍用毫米，内部 ×0.001 再采样/回退。
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

    // 模型为 m：5 mm → 0.005 m，100 mm → 0.1 m
    const Standard_Real mmToModel = 0.001;
    const Standard_Real spacing = spacingMm * mmToModel;
    const Standard_Real retract = retractMm * mmToModel;

    if (spacing <= Precision::Confusion()) {
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

    GCPnts_UniformAbscissa sampler(curve, spacing);
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

    if (retract > Precision::Confusion()) {
        const DiscretePoint& seamFirst = trajectory.front();
        const DiscretePoint& seamLast = trajectory.back();

        DiscretePoint startArc = seamFirst;
        startArc.position =
            seamFirst.position.Translated(gp_Vec(seamFirst.zDir) * (-retract));

        DiscretePoint endArc = seamLast;
        endArc.position =
            seamLast.position.Translated(gp_Vec(seamLast.zDir) * (-retract));

        trajectory.insert(trajectory.begin(), startArc);
        trajectory.push_back(endArc);
    }

    return Standard_True;
}
