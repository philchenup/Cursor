#ifndef DISPLAY_CLOUD_AND_AXES_H
#define DISPLAY_CLOUD_AND_AXES_H

#include <AIS_InteractiveContext.hxx>
#include <AIS_PointCloud.hxx>
#include <AIS_Trihedron.hxx>
#include <Aspect_TypeOfMarker.hxx>
#include <Geom_Axis2Placement.hxx>
#include <Graphic3d_ArrayFlags.hxx>
#include <Graphic3d_ArrayOfPoints.hxx>
#include <Prs3d_PointAspect.hxx>
#include <Quantity_Color.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <Eigen/Geometry>

/**
 * @brief 在 OCCT 7.9 上下文中显示点云以及两个 Affine3f 对应的坐标轴。
 *
 * 点云坐标系已是相机系，pose 不再做相机变换。
 * 显示后需由调用方 Redraw（例如 myOccView->Redraw()）。
 *
 * @param ctx         AIS 上下文（myOccView->getContext()）
 * @param cloud       点云指针（需支持 size()/empty()/points[i].{x,y,z,r,g,b}，如 ct::Cloud::Ptr）
 * @param poseStart   起点位姿，列向量为 X/Y/Z 轴
 * @param poseEnd     终点位姿
 * @param axisLength  坐标轴长度
 */
template <class CloudPtr>
void DisplayCloudAndAxes(const Handle(AIS_InteractiveContext)& ctx,
                         const CloudPtr& cloud,
                         const Eigen::Affine3f& poseStart,
                         const Eigen::Affine3f& poseEnd,
                         Standard_Real axisLength = 20.0)
{
    if (ctx.IsNull()) {
        return;
    }

    auto makeAxes = [axisLength](const Eigen::Affine3f& pose) -> Handle(AIS_Trihedron) {
        const Eigen::Vector3f t = pose.translation();
        const Eigen::Matrix3f R = pose.linear();
        const gp_Ax2 ax(gp_Pnt(t.x(), t.y(), t.z()),
                        gp_Dir(R(0, 2), R(1, 2), R(2, 2)),
                        gp_Dir(R(0, 0), R(1, 0), R(2, 0)));
        Handle(AIS_Trihedron) tri = new AIS_Trihedron(new Geom_Axis2Placement(ax));
        tri->SetSize(axisLength);
        tri->SetDrawArrows(Standard_True);
        return tri;
    };

    if (cloud && !cloud->empty()) {
        const Standard_Integer n = static_cast<Standard_Integer>(cloud->size());
        Handle(Graphic3d_ArrayOfPoints) pts =
            new Graphic3d_ArrayOfPoints(n, Graphic3d_ArrayFlags_VertexColor);
        for (Standard_Integer i = 0; i < n; ++i) {
            const auto& p = cloud->points[i];
            pts->AddVertex(gp_Pnt(p.x, p.y, p.z),
                           Quantity_Color(p.r / 255.0, p.g / 255.0, p.b / 255.0, Quantity_TOC_sRGB));
        }
        Handle(AIS_PointCloud) aisCloud = new AIS_PointCloud();
        aisCloud->SetPoints(pts);
        aisCloud->Attributes()->SetPointAspect(
            new Prs3d_PointAspect(Aspect_TOM_POINT, Quantity_NOC_WHITE, 2.0));
        ctx->Display(aisCloud, Standard_False);
    }

    ctx->Display(makeAxes(poseStart), Standard_False);
    ctx->Display(makeAxes(poseEnd), Standard_False);
}

#endif // DISPLAY_CLOUD_AND_AXES_H
