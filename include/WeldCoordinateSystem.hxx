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
//! X = travel (edge tangent), Z = 45° down vs world XOY and ⊥ X, Y = Z × X.
struct DiscretePoint {
  gp_Pnt position; // point on the edge
  gp_Dir xDir;     // X: tangent / travel direction
  gp_Dir yDir;     // Y: zDir × xDir
  gp_Dir zDir;     // Z: 45° down to world XOY, perpendicular to X
};

//! Compute a weld local coordinate system from a selected edge (mid-point sample).
//!
//! Legacy convention kept for callers that still use Y-along-edge:
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
//! - xDir: edge tangent (edge orientation)
//! - zDir: unit direction ⊥ xDir, angle with world XOY plane = 45°, pointing down;
//!         when two solutions exist, the one closer to the face-normal bisector is kept
//! - yDir: zDir × xDir  (right-handed: x × y = z)
//!
//! @param selectShape  solid / shell that owns the edge
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
