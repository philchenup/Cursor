// Example: discretize a weld edge into DiscretePoint samples.
// Requires OpenCASCADE 7.4+.

#include "WeldCoordinateSystem.hxx"

#include <BRepPrimAPI_MakeBox.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <iostream>

int main()
{
  const TopoDS_Shape selectShape = BRepPrimAPI_MakeBox(100.0, 50.0, 30.0).Shape();

  TopoDS_Edge edge;
  for (TopExp_Explorer exp(selectShape, TopAbs_EDGE); exp.More(); exp.Next()) {
    edge = TopoDS::Edge(exp.Current());
    break;
  }

  // Explicit inputs: spacing / reverseZ / retract (起弧收弧后退距离).
  WeldDiscretizeOptions opt;
  opt.spacingMm = 10.0;
  opt.reverseZ  = Standard_False;
  opt.retractMm = 50.0; // 起弧点、收弧点沿 -Z 后退 50mm

  // Optional: normalize edge so start.Y >= end.Y before discretization
  // (DiscretizeWeldTrajectory also does this internally).
  const TopoDS_Edge orientedEdge = OrientEdgeStartYNotLessThanEndY(edge);

  std::vector<DiscretePoint> trajectory;
  if (!DiscretizeWeldTrajectory(selectShape, orientedEdge, trajectory, opt)) {
    std::cerr << "Failed to build weld trajectory.\n";
    return 1;
  }

  std::cout << "trajectory size: " << trajectory.size()
            << " (front=起弧, back=收弧), retractMm=" << opt.retractMm << '\n';
  for (std::size_t i = 0; i < trajectory.size(); ++i) {
    const DiscretePoint& s = trajectory[i];
    const char* tag = "";
    if (i == 0) {
      tag = " 起弧";
    }
    else if (i + 1 == trajectory.size()) {
      tag = " 收弧";
    }
    std::cout << "[" << i << "]" << tag << " p=("
              << s.position.X() << ", " << s.position.Y() << ", " << s.position.Z()
              << ") x=(" << s.xDir.X() << ", " << s.xDir.Y() << ", " << s.xDir.Z()
              << ") y=(" << s.yDir.X() << ", " << s.yDir.Y() << ", " << s.yDir.Z()
              << ") z=(" << s.zDir.X() << ", " << s.zDir.Y() << ", " << s.zDir.Z()
              << ")\n";
  }
  return 0;
}
