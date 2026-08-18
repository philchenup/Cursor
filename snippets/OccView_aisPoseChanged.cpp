// occView.h signals 增加：
//   void aisPoseChanged(AIS_Shape* ais);     // 拖动过程中，只同步这一个
//   void aisDragFinished(AIS_Shape* ais);    // 松开并烘焙几何后，只重绑这一个

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

        Handle(AIS_Shape) aisShape =
            Handle(AIS_Shape)::DownCast(myManipulatorTarget);

        if (!aisShape.IsNull())
        {
            const gp_Trsf& trsf = aisShape->LocalTransformation();
            if (trsf.Form() != gp_Identity)
            {
                TopoDS_Shape movedShape =
                    BRepBuilderAPI_Transform(aisShape->Shape(), trsf, Standard_False).Shape();
                aisShape->SetShape(movedShape);
                aisShape->ResetTransformation();
                myContext->Redisplay(aisShape, Standard_False);

                myManipulator->Detach();
                myManipulator->Attach(aisShape);
                myManipulator->SetSize(120.0f);
                myManipulator->SetModeActivationOnDetection(Standard_True);

                emit aisDragFinished(aisShape.get());
            }
            else
            {
                emit aisPoseChanged(aisShape.get());
            }
        }

        myView->Redraw();
        return;
    }
    // ... 其余原逻辑不变
}
