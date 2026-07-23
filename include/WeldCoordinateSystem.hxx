#ifndef WELD_COORDINATE_SYSTEM_HXX
#define WELD_COORDINATE_SYSTEM_HXX

#include <Standard_Boolean.hxx>
#include <Standard_Real.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Ax3.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <vector>

//! One sample on the weld path with a right-handed local frame.
//! Y = edge tangent, Z = adjacent-face normal bisector, X = Y × Z.
struct DiscretePoint {
  gp_Pnt position; // point on the edge
  gp_Dir xDir;     // X: right-hand rule, X = Y × Z
  gp_Dir yDir;     // Y: edge tangent / travel direction
  gp_Dir zDir;     // Z: unit bisector of the two adjacent face normals
};

//! Compute a weld local coordinate system from a selected edge (mid-point sample).
//!
//! - Origin: midpoint of the edge
//! - Y axis: tangent along the edge orientation
//! - Z axis: unit bisector of the two adjacent face normals
//! - X axis: right-hand rule, X = Y × Z
Standard_Boolean ComputeWeldCoordinateSystem(
    const TopoDS_Shape& selectShape,
    const TopoDS_Edge&   edge,
    gp_Ax3&             weldAxis);

//! Discretize the selected weld edge by arc length and build a trajectory.
//!
//! At each sample:
//! - position: point on the edge
//! - yDir: edge tangent (edge orientation)
//! - zDir: unit bisector of the two adjacent face normals (⊥ yDir)
//! - xDir: yDir × zDir  (right-handed: x × y = z)
//!
//! @param selectShape  solid / shell / compound that owns the edge
//! @param edge         selected weld edge
//! @param trajectory   output samples (cleared on entry)
//! @param spacingMm    arc-length step in model units (default 10 mm)
//! @return Standard_True if at least one valid sample was produced
Standard_Boolean DiscretizeWeldTrajectory(
    const TopoDS_Shape&        selectShape,
    const TopoDS_Edge&          edge,
    std::vector<DiscretePoint>& trajectory,
    Standard_Real               spacingMm = 10.0);

#endif // WELD_COORDINATE_SYSTEM_HXX
