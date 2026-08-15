void MainWindow::showToolPoints(const std::vector<rl::math::Vector3>& toolPoints)
{
	if (this->myOccView == nullptr) {
		return;
	}
	if (toolPoints.empty()) {
		return;
	}

	// 模型单位为 m：线径 5 mm → 半径 2.5 mm = 0.0025 m
	const Standard_Real radius = 0.0025;

	BRep_Builder builder;
	TopoDS_Compound compound;
	builder.MakeCompound(compound);

	for (std::size_t i = 0; i < toolPoints.size(); ++i) {
		const gp_Pnt p(toolPoints[i](0), toolPoints[i](1), toolPoints[i](2));

		Handle(Geom_CartesianPoint) aGeomPnt = new Geom_CartesianPoint(p);
		Handle(AIS_Point) anAISPoint = new AIS_Point(aGeomPnt);
		Handle(Prs3d_PointAspect) aPointAspect =
			new Prs3d_PointAspect(Aspect_TOM_BALL, Quantity_NOC_GREEN, 2.0);
		anAISPoint->SetColor(Quantity_NOC_GREEN);
		anAISPoint->Attributes()->SetPointAspect(aPointAspect);
		this->myOccView->getContext()->Display(anAISPoint, Standard_False);

		if (i + 1 >= toolPoints.size()) {
			continue;
		}
		const gp_Pnt q(toolPoints[i + 1](0), toolPoints[i + 1](1), toolPoints[i + 1](2));
		const Standard_Real len = p.Distance(q);
		if (len <= Precision::Confusion()) {
			continue;
		}
		const gp_Ax2 ax(p, gp_Dir(gp_Vec(p, q)));
		builder.Add(compound, BRepPrimAPI_MakeCylinder(ax, radius, len).Shape());
	}

	if (toolPoints.size() >= 2) {
		Handle(AIS_Shape) aLine = new AIS_Shape(compound);
		aLine->SetColor(Quantity_NOC_GREEN);
		aLine->SetDisplayMode(AIS_Shaded);
		this->myOccView->getContext()->Display(aLine, Standard_False);
	}
	this->myOccView->Redraw();
}
