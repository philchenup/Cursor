// 放进 OccView 点击 / 选择回调里：点到 ViewCube、坐标系、点标记等
// 非 BRep 对象时 DetectedShape() 为空，直接 ShapeType() 会崩。

void OccView::onSelectionChanged() // 或 mouseRelease 里 Select 之后
{
	Handle(AIS_InteractiveContext) ctx = myContext;
	if (ctx.IsNull())
		return;

	if (!ctx->HasDetected())
		return;

	Handle(AIS_InteractiveObject) obj = ctx->DetectedInteractive();
	if (obj.IsNull())
		return;

	// 导航立方、焊缝坐标系、点标记：不是 TopoDS_Shape，不能取 ShapeType
	if (obj->IsKind(STANDARD_TYPE(AIS_ViewCube))
		|| obj->IsKind(STANDARD_TYPE(AIS_Trihedron))
		|| obj->IsKind(STANDARD_TYPE(AIS_Point)))
	{
		return;
	}

	if (!ctx->HasDetectedShape())
		return;

	const TopoDS_Shape& shape = ctx->DetectedShape();
	if (shape.IsNull())
		return;

	const TopAbs_ShapeEnum type = shape.ShapeType();
	(void)type;
}

// 若用的是 SelectedShape()（Select 之后）：
void OccView::handleSelectedShape()
{
	Handle(AIS_InteractiveContext) ctx = myContext;
	if (ctx.IsNull())
		return;

	for (ctx->InitSelected(); ctx->MoreSelected(); ctx->NextSelected())
	{
		Handle(AIS_Shape) aisShape =
			Handle(AIS_Shape)::DownCast(ctx->SelectedInteractive());
		if (aisShape.IsNull())
			continue;

		const TopoDS_Shape shape = aisShape->Shape();
		if (shape.IsNull())
			continue;

		const TopAbs_ShapeEnum type = shape.ShapeType();
		(void)type;
	}
}
