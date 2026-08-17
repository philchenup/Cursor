void MainWindow::showWeldTrajPt(const std::vector<DiscretePoint>& trajPt) {
	if (trajPt.size() < 2) return;
	if (this->myOccView == nullptr) return;

	Handle(AIS_InteractiveContext) ctx = this->myOccView->getContext();

	for (const auto& info : trajPt)
	{
		// 用 Trihedron 显示局部坐标系
		// gp_Ax2 需要 (origin, mainDir=Z, xDir)
		gp_Ax2 anAx2(info.position, info.zDir, info.xDir);
		Handle(Geom_Axis2Placement) anAxisPlacement =
			new Geom_Axis2Placement(anAx2);

		Handle(AIS_Trihedron) aTrihedron = new AIS_Trihedron(anAxisPlacement);
		aTrihedron->SetSize(0.005);

		// 设置坐标轴颜色：X-红 Y-绿 Z-蓝
		Handle(Prs3d_DatumAspect) aDatumAspect = new Prs3d_DatumAspect();
		aDatumAspect->LineAspect(Prs3d_DatumParts_XAxis)
			->SetColor(Quantity_NOC_RED);
		aDatumAspect->LineAspect(Prs3d_DatumParts_YAxis)
			->SetColor(Quantity_NOC_GREEN);
		aDatumAspect->LineAspect(Prs3d_DatumParts_ZAxis)
			->SetColor(Quantity_NOC_BLUE1);
		aTrihedron->Attributes()->SetDatumAspect(aDatumAspect);
		aDatumAspect->SetAxisLength(0.1, 0.1, 0.1);

		// 同时显示一个点标记（可选）
		Handle(Geom_CartesianPoint) aGeomPnt =
			new Geom_CartesianPoint(info.position);
		Handle(AIS_Point) anAISPoint = new AIS_Point(aGeomPnt);
		Handle(Prs3d_PointAspect) aPointAspect = new Prs3d_PointAspect(
			Aspect_TOM_O_PLUS, Quantity_NOC_YELLOW, 0.002);
		anAISPoint->Attributes()->SetPointAspect(aPointAspect);

		// selectionMode = -1：只显示、不激活选择。AIS_Trihedron / AIS_Point
		// 没有 TopoDS_TShape，若可被点中，点击回调里 ShapeType() 会空指针崩溃。
		ctx->Display(aTrihedron, 0, -1, Standard_False);
		ctx->Deactivate(aTrihedron);
		ctx->Display(anAISPoint, 0, -1, Standard_False);
		ctx->Deactivate(anAISPoint);
	}
	ctx->UpdateCurrentViewer();
}
