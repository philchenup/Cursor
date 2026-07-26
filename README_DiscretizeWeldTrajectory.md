# DiscretizeWeldTrajectory: Y along edge orientation

## I/O

Unchanged.

## Frame at each sample

| Axis | Direction |
|------|-----------|
| **Y** | edge tangent, **same sense** as `TopoDS_Edge` (`FirstVertex` → `LastVertex`) |
| **Z** | ± unit bisector of the two adjacent face normals (⊥ Y) |
| **X** | `Y × Z` (right-handed: `X × Y = Z`) |

If `BRepAdaptor_Curve::D1` comes out anti-parallel to the topological edge sense, the tangent is flipped so **Y matches the edge direction** (not the opposite).
