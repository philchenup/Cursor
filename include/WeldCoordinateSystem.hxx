#ifndef WELD_COORDINATE_SYSTEM_HXX
#define WELD_COORDINATE_SYSTEM_HXX

#include <Standard_Boolean.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Ax3.hxx>

//! Compute a weld local coordinate system from a selected edge.
//!
//! Convention (OpenCASCADE 7.4+):
//! - Origin: midpoint of the edge
//! - Y axis: tangent along the edge orientation
//! - Z axis: unit bisector of the two adjacent face normals
//!           (exterior / open-angle side of the dihedral)
//! - X axis: right-hand rule, X = Y × Z
//!
//! @param selectShape  solid / shell / face compound that owns the edge
//! @param edge         selected weld edge (orientation defines +Y)
//! @param weldAxis     resulting right-handed local frame (gp_Ax3)
//! @return Standard_True on success; Standard_False if the edge is not
//!         shared by exactly two faces or the frame is degenerate
Standard_Boolean ComputeWeldCoordinateSystem(
    const TopoDS_Shape& selectShape,
    const TopoDS_Edge&   edge,
    gp_Ax3&             weldAxis);

#endif // WELD_COORDINATE_SYSTEM_HXX
