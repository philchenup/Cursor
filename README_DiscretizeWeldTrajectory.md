# DiscretizeWeldTrajectory: X along edge tangent

## I/O

Unchanged.

## Frame at each sample

| Axis | Direction |
|------|-----------|
| **X** | edge tangent (travel / edge orientation) |
| **Z** | ± unit bisector of the two adjacent face normals (⊥ X) |
| **Y** | `Z × X` (right-handed: `X × Y = Z`) |

Previously Y was the edge tangent and `X = Y × Z`. X/Y are swapped so X follows the seam.
