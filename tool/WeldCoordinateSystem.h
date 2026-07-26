#ifndef WELD_COORDINATE_SYSTEM_HXX
#define WELD_COORDINATE_SYSTEM_HXX
#include "GlobalDefs.h"

Standard_Boolean ComputeWeldCoordinateSystem(
    const TopoDS_Shape& selectShape,
    const TopoDS_Edge& edge,
    gp_Ax3& weldAxis,
    Standard_Boolean    reverseZ = Standard_False);
//! Discretize the selected weld edge by arc length and build a trajectory.
//!
//! At each sample:
//! - position: point on the edge
//! - xDir: edge tangent (edge orientation)
//! - zDir: unit bisector of the two adjacent face normals (⊥ xDir)
//! - yDir: zDir × xDir  (right-handed: x × y = z)
//!
//! @param selectShape  solid / shell / compound that owns the edge
//! @param edge         selected weld edge
//! @param trajectory   output samples (cleared on entry)
//! @param spacingMm    arc-length step in model units (default 10 mm)
//! @param reverseZ     Standard_False: Z along face-normal bisector;
//!                     Standard_True:  Z along the opposite bisector
//! @return Standard_True if at least one valid sample was produced
Standard_Boolean DiscretizeWeldTrajectory(
    const TopoDS_Shape& selectShape,
    const TopoDS_Edge& edge,
    std::vector<DiscretePoint>& trajectory,
    Standard_Real               spacingMm = 10.0,
    Standard_Boolean            reverseZ = Standard_False,
    Standard_Real               retractMm = 50.0);
#endif // WELD_COORDINATE_SYSTEM_HXX
