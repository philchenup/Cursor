// occView.h signals：
//   void aisPoseChanged(AIS_Shape* ais);   // 拖动过程中，只同步这一个
//   void aisDragFinished(AIS_Shape* ais);  // 松开并烘焙几何后，只重绑这一个
//
// 需要：AIS_Manipulator、AIS_Shape、BRepBuilderAPI_Transform、gp_Trsf。
// MainWindow 连接这两路信号；OccView 不调用 bind / sync / unbind。

void OccView::onMouseMove(const int theFlags, const QPoint thePoint)
{
	if (myIsManipulating && !myManipulator.IsNull())
	{
		myManipulator->Transform(thePoint.x(), thePoint.y(), myView);
		myView->Redraw();
		Handle(AIS_Shape) dragged =
			Handle(AIS_Shape)::DownCast(myManipulatorTarget);
		if (!dragged.IsNull())
			emit aisPoseChanged(dragged.get());
		return;
	}
	// ... 其余原逻辑不变
}

void OccView::onLButtonUp(const int theFlags, const QPoint thePoint)
{
	if (myIsManipulating && !myManipulator.IsNull())
	{
		myManipulator->StopTransform(Standard_True);
		myIsManipulating = Standard_False;

		Handle(AIS_Shape) loadShape =
			Handle(AIS_Shape)::DownCast(myManipulatorTarget);

		if (!loadShape.IsNull())
		{
			const gp_Trsf trsf = loadShape->LocalTransformation();
			if (trsf.Form() != gp_Identity)
			{
				TopoDS_Shape movedShape =
					BRepBuilderAPI_Transform(loadShape->Shape(), trsf, Standard_False).Shape();
				loadShape->SetShape(movedShape);
				loadShape->ResetTransformation();
				myContext->Redisplay(loadShape, Standard_False);

				myManipulator->Detach();
				myManipulator->Attach(loadShape);
				myManipulator->SetSize(120.0f);
				myManipulator->SetModeActivationOnDetection(Standard_True);

				emit aisDragFinished(loadShape.get());
			}
			else
			{
				emit aisPoseChanged(loadShape.get());
			}
		}

		myView->Redraw();
		return;
	}
	// ... 其余原逻辑不变
}
