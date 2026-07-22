// Example: build a weld local frame from a user-selected edge.
// Requires OpenCASCADE 7.4 or newer.
//
// Link against: TKernel TKMath TKBRep TKG3d TKG2d TKGeomBase TKTopAlgo ...

#include "WeldCoordinateSystem.hxx"

#include <BRepPrimAPI_MakeBox.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <iostream>

int main()
{
  // Demo solid — replace with the shape that owns the selected edge.
  const TopoDS_Shape selectShape = BRepPrimAPI_MakeBox(100.0, 50.0, 30.0).Shape();

  // Demo: pick the first edge; in an application this comes from selection.
  TopoDS_Edge edge;
  for (TopExp_Explorer exp(selectShape, TopAbs_EDGE); exp.More(); exp.Next()) {
    edge = TopoDS::Edge(exp.Current());
    break;
  }

  gp_Ax3 weldAxis;
  if (!ComputeWeldCoordinateSystem(selectShape, edge, weldAxis)) {
    std::cerr << "Failed to compute weld coordinate system "
                 "(edge must be shared by exactly two faces).\n";
    return 1;
  }

  const gp_Pnt o = weldAxis.Location();
  const gp_Dir x = weldAxis.XDirection();
  const gp_Dir y = weldAxis.YDirection();
  const gp_Dir z = weldAxis.Direction(); // main axis = Z

  std::cout << "Weld CS origin : " << o.X() << ", " << o.Y() << ", " << o.Z() << '\n'
            << "X (right-hand) : " << x.X() << ", " << x.Y() << ", " << x.Z() << '\n'
            << "Y (edge)       : " << y.X() << ", " << y.Y() << ", " << y.Z() << '\n'
            << "Z (bisector)   : " << z.X() << ", " << z.Y() << ", " << z.Z() << '\n';

  return 0;
}
