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

//! Default 起弧/收弧 retract distance along -Z (mm).
const Standard_Real kDefaultArcRetractMm = 50.0;

//! Default seam sample spacing along the edge (mm).
const Standard_Real kDefaultSeamSpacingMm = 10.0;

//! One sample on the weld path with a right-handed local frame.
//! Y = edge tangent, Z = adjacent-face normal bisector (±), X = Y × Z.
struct DiscretePoint {
  gp_Pnt position; // point on the edge (or retracted arc point)
  gp_Dir xDir;     // X: right-hand rule, X = Y × Z
  gp_Dir yDir;     // Y: edge tangent / travel direction
  gp_Dir zDir;     // Z: unit bisector (±) of the two adjacent face normals
};

//! Parameters for seam discretization / arc retract.
struct WeldDiscretizeOptions {
  Standard_Real    spacingMm = kDefaultSeamSpacingMm; // 轨迹插值间距 (mm)
  Standard_Boolean reverseZ  = Standard_False;        // true: Z 取二分角反向
  Standard_Real    retractMm = kDefaultArcRetractMm;  // 起弧/收弧沿 -Z 后退距离 (mm); 0 关闭
};

//! If the oriented start point has smaller Y than the end point, reverse the edge.
//! Otherwise return @p edge unchanged.
//!
//! After this call, travel along the edge goes from larger (or equal) Y to smaller Y.
TopoDS_Edge OrientEdgeStartYNotLessThanEndY(const TopoDS_Edge& edge);

//! Compute a weld local coordinate system from a selected edge (mid-point sample).
//!
//! - Origin: midpoint of the edge
//! - Y axis: tangent along the edge orientation
//! - Z axis: unit bisector of the two adjacent face normals
//! - X axis: right-hand rule, X = Y × Z
//!
//! @param reverseZ  Standard_False: Z along face-normal bisector;
//!                  Standard_True:  Z along the opposite bisector
Standard_Boolean ComputeWeldCoordinateSystem(
    const TopoDS_Shape& selectShape,
    const TopoDS_Edge&   edge,
    gp_Ax3&             weldAxis,
    Standard_Boolean    reverseZ = Standard_False);

//! Discretize the selected weld edge and build a trajectory.
//!
//! Seam samples use @p options.spacingMm. When @p options.retractMm > 0:
//! - front: 起弧点 = first seam point − retractMm * zDir
//! - back:  收弧点 = last  seam point − retractMm * zDir
//!
//! Example:
//! @code
//!   WeldDiscretizeOptions opt;
//!   opt.spacingMm = 10.0;
//!   opt.reverseZ  = Standard_False;
//!   opt.retractMm = 50.0;   // <-- 起弧/收弧后退距离
//!   DiscretizeWeldTrajectory(selectShape, edge, trajectory, opt);
//! @endcode
Standard_Boolean DiscretizeWeldTrajectory(
    const TopoDS_Shape&          selectShape,
    const TopoDS_Edge&            edge,
    std::vector<DiscretePoint>&   trajectory,
    const WeldDiscretizeOptions&  options = WeldDiscretizeOptions());

#endif // WELD_COORDINATE_SYSTEM_HXX
