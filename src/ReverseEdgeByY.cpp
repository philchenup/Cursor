#include "ReverseEdgeByY.h"

#include <BRep_Tool.hxx>
#include <TopExp.hxx>
#include <TopoDS.hxx>
#include <gp_Pnt.hxx>

TopoDS_Edge ReverseEdgeIfStartYLessThanEndY(const TopoDS_Edge& edge)
{
  if (edge.IsNull())
  {
    return edge;
  }

  // FirstVertex / LastVertex 会考虑边的 Orientation
  const TopoDS_Vertex vFirst = TopExp::FirstVertex(edge, Standard_True);
  const TopoDS_Vertex vLast  = TopExp::LastVertex(edge, Standard_True);

  if (vFirst.IsNull() || vLast.IsNull())
  {
    return edge;
  }

  const gp_Pnt pStart = BRep_Tool::Pnt(vFirst);
  const gp_Pnt pEnd   = BRep_Tool::Pnt(vLast);

  // 起点 Y < 终点 Y 时首尾反转
  if (pStart.Y() < pEnd.Y())
  {
    return TopoDS::Edge(edge.Reversed());
  }

  return edge;
}
