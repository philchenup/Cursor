#pragma once

#include "GlobalDefs.h"

#include <Precision.hxx>

#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

class WeldingPathBuilder
{
public:
    WeldingPathBuilder(double standoff = 3.0,
        double coincidenceTol = Precision::Confusion() * 10.0,
        double gapFitTangentScale = 5.0)
        : m_standoff(standoff),
        m_tol(coincidenceTol),
        m_gapScale(gapFitTangentScale) {
    }

    void GetCirclePath(const std::vector<ProcessedEdge>& edges, TopoDS_Wire& wires);

    bool SortEdges(const std::vector<TopoDS_Edge>& weldEdges,
        std::vector<ProcessedEdge>& sortedEdges,
        std::vector<EdgeCluster>& sortedClusters);

    bool BuildWeldingTrajectory(const TopoDS_Shape& selectShape,
        const std::vector<EdgeCluster>& sortedClusters,
        std::vector<DiscretePoint>& trajectory,
        double spacingMm = 10.0,
        double retractMm = 50.0,
        double verticalTiltDeg = 45.0,
        std::vector<std::vector<DiscretePoint>>* clusterTrajectories = nullptr);

    std::vector<DiscretePoint> DiscretizeWireWithCustomFrame(
        const TopoDS_Wire& wire,
        const TopoDS_Shape& parentShape,
        double stepSize,
        double tolerance,
        double maxAngleStep = 5.0 * M_PI / 180.0);

private:
    double m_standoff;
    double m_tol;
    double m_gapScale;

    bool GetTangentAt(const ProcessedEdge& pe, gp_Vec& tangent);

    bool BuildInterpolationCurve(const ProcessedEdge& pe1,
        const ProcessedEdge& pe2,
        TopoDS_Edge& outEdge);

    TopoDS_Wire BuildWiresFromUnorderedEdges(const std::vector<TopoDS_Edge>& edges, double tolerance = 1e-6);

    bool BuildFilletArc(const ProcessedEdge& pe1, const ProcessedEdge& pe2, TopoDS_Edge& filletEdge);

    bool IsVerticalByEndpoints(const TopoDS_Edge& edge, double vecEps);

    bool ComputeFaceNormalAt(const TopoDS_Face& face,
        const gp_Pnt& p,
        double vecEps,
        gp_Vec& outNormal);

    bool BuildClusterTrajectory(const TopoDS_Shape& selectShape,
        const EdgeCluster& cluster,
        double spacingMm,
        double retractMm,
        double verticalTiltDeg,
        std::vector<DiscretePoint>& out);
};

Standard_Boolean ComputeWeldCoordinateSystem(
    const TopoDS_Shape& selectShape,
    const TopoDS_Edge& edge,
    gp_Ax3& weldAxis,
    Standard_Boolean reverseZ = Standard_False);

Standard_Boolean DiscretizeWeldTrajectory(
    const TopoDS_Shape& selectShape,
    const TopoDS_Edge& edge,
    std::vector<DiscretePoint>& trajectory,
    Standard_Real spacingMm = 10.0,
    Standard_Boolean reverseZ = Standard_False,
    Standard_Real retractMm = 50.0);
