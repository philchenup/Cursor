#ifndef REVERSE_EDGE_BY_Y_H
#define REVERSE_EDGE_BY_Y_H

#include <TopoDS_Edge.hxx>

/**
 * @brief 若边的起点 Y 小于终点 Y，则首尾反转该边。
 *
 * 比较时考虑边自身的 Orientation。
 * 反转后保证起点 Y >= 终点 Y（Y 相等时保持原方向）。
 *
 * @param edge 输入边（const，不会原地修改）
 * @return 必要时已反转的边；否则返回原边副本
 */
TopoDS_Edge ReverseEdgeIfStartYLessThanEndY(const TopoDS_Edge& edge);

#endif // REVERSE_EDGE_BY_Y_H
