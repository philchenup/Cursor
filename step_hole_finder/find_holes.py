"""
Detect cylindrical holes in a STEP model using OpenCASCADE (OCP / CadQuery).

Approach:
1. Read STEP → B-Rep solid
2. Traverse faces and keep cylindrical surfaces
3. Classify hole vs boss by testing whether a point near the cylinder axis
   lies outside the solid (void = hole)
4. Cluster coaxial cylinders of similar radius into one logical hole
5. Filter by target diameter (default 8 mm) with tolerance
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable, List, Optional, Sequence, Tuple

from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.BRepClass3d import BRepClass3d_SolidClassifier
from OCP.BRepGProp import BRepGProp
from OCP.GeomAbs import GeomAbs_Cylinder
from OCP.GProp import GProp_GProps
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPControl import STEPControl_Reader
from OCP.TopAbs import TopAbs_FACE, TopAbs_IN, TopAbs_OUT, TopAbs_REVERSED
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS, TopoDS_Face, TopoDS_Shape
from OCP.gp import gp_Ax1, gp_Dir, gp_Pnt, gp_Vec


@dataclass(frozen=True)
class HoleCandidate:
    """One logical cylindrical hole (possibly merged from several faces)."""

    diameter_mm: float
    radius_mm: float
    axis_point: Tuple[float, float, float]
    axis_direction: Tuple[float, float, float]
    depth_mm: Optional[float]
    face_count: int
    area_mm2: float

    def to_dict(self) -> dict:
        return asdict(self)


def load_step(path: str | Path) -> TopoDS_Shape:
    """Load a STEP file and return the root TopoDS shape."""
    path = Path(path)
    if not path.is_file():
        raise FileNotFoundError(f"STEP file not found: {path}")

    reader = STEPControl_Reader()
    status = reader.ReadFile(str(path))
    if status != IFSelect_RetDone:
        raise RuntimeError(f"Failed to read STEP file: {path}")

    reader.TransferRoots()
    shape = reader.OneShape()
    if shape.IsNull():
        raise RuntimeError(f"STEP file contains no transferable shape: {path}")
    return shape


def _iter_faces(shape: TopoDS_Shape) -> Iterable[TopoDS_Face]:
    explorer = TopExp_Explorer(shape, TopAbs_FACE)
    while explorer.More():
        yield TopoDS.Face_s(explorer.Current())
        explorer.Next()


def _unit(v: gp_Dir | gp_Vec) -> Tuple[float, float, float]:
    return (float(v.X()), float(v.Y()), float(v.Z()))


def _point(p: gp_Pnt) -> Tuple[float, float, float]:
    return (float(p.X()), float(p.Y()), float(p.Z()))


def _face_area(face: TopoDS_Face) -> float:
    props = GProp_GProps()
    BRepGProp.SurfaceProperties_s(face, props)
    return abs(props.Mass())


def _cylinder_height_estimate(face: TopoDS_Face, adaptor: BRepAdaptor_Surface) -> float:
    """Estimate axial length of a cylindrical face from UV bounds."""
    u_mid = 0.5 * (adaptor.FirstUParameter() + adaptor.LastUParameter())
    v0 = adaptor.FirstVParameter()
    v1 = adaptor.LastVParameter()
    p0 = adaptor.Value(u_mid, v0)
    p1 = adaptor.Value(u_mid, v1)
    return p0.Distance(p1)


def _project_point_to_axis(point: gp_Pnt, axis: gp_Ax1) -> gp_Pnt:
    origin = axis.Location()
    direction = axis.Direction()
    vec = gp_Vec(origin, point)
    t = vec.Dot(gp_Vec(direction.XYZ()))
    return gp_Pnt(
        origin.X() + direction.X() * t,
        origin.Y() + direction.Y() * t,
        origin.Z() + direction.Z() * t,
    )


def _is_hole_cylinder(shape: TopoDS_Shape, face: TopoDS_Face, adaptor: BRepAdaptor_Surface) -> bool:
    """
    Return True if the cylindrical face bounds a cavity (hole), not a boss.

    Classify a point on the cylinder axis at the face mid-height:
    - outside the solid  → cavity / hole
    - inside the solid   → material / boss
    """
    u = 0.5 * (adaptor.FirstUParameter() + adaptor.LastUParameter())
    v = 0.5 * (adaptor.FirstVParameter() + adaptor.LastVParameter())
    point_on_face = adaptor.Value(u, v)
    axis_point = _project_point_to_axis(point_on_face, adaptor.Cylinder().Axis())

    classifier = BRepClass3d_SolidClassifier(shape, axis_point, 1e-6)
    state = classifier.State()
    if state == TopAbs_OUT:
        return True
    if state == TopAbs_IN:
        return False
    # Ambiguous (ON): fall back to B-Rep orientation; inner faces are often REVERSED.
    return face.Orientation() == TopAbs_REVERSED


def _axes_compatible(
    ax1: gp_Ax1,
    ax2: gp_Ax1,
    radius1: float,
    radius2: float,
    *,
    radius_tol: float,
    angle_tol_deg: float,
    distance_tol: float,
) -> bool:
    if abs(radius1 - radius2) > radius_tol:
        return False

    d1 = ax1.Direction()
    d2 = ax2.Direction()
    # Parallel if |dot| ≈ 1
    dot = abs(d1.Dot(d2))
    if dot < math.cos(math.radians(angle_tol_deg)):
        return False

    # Distance between skew/parallel axes: |(P2-P1) × dir|
    p1 = ax1.Location()
    p2 = ax2.Location()
    vec = gp_Vec(p1, p2)
    cross = vec.Crossed(gp_Vec(d1))
    dist = cross.Magnitude()
    return dist <= distance_tol


def _merge_coaxial(
    cylinders: Sequence[dict],
    *,
    radius_tol: float,
    angle_tol_deg: float,
    distance_tol: float,
) -> List[HoleCandidate]:
    clusters: List[List[dict]] = []
    for cyl in cylinders:
        placed = False
        for cluster in clusters:
            ref = cluster[0]
            if _axes_compatible(
                ref["axis"],
                cyl["axis"],
                ref["radius"],
                cyl["radius"],
                radius_tol=radius_tol,
                angle_tol_deg=angle_tol_deg,
                distance_tol=distance_tol,
            ):
                cluster.append(cyl)
                placed = True
                break
        if not placed:
            clusters.append([cyl])

    holes: List[HoleCandidate] = []
    for cluster in clusters:
        radius = sum(c["radius"] for c in cluster) / len(cluster)
        area = sum(c["area"] for c in cluster)
        # Prefer the face with largest area for axis reference.
        ref = max(cluster, key=lambda c: c["area"])
        depth = max((c["height"] for c in cluster), default=None)
        loc = ref["axis"].Location()
        direction = ref["axis"].Direction()
        holes.append(
            HoleCandidate(
                diameter_mm=radius * 2.0,
                radius_mm=radius,
                axis_point=_point(loc),
                axis_direction=_unit(direction),
                depth_mm=depth,
                face_count=len(cluster),
                area_mm2=area,
            )
        )
    return holes


def find_cylindrical_holes(
    shape: TopoDS_Shape,
    *,
    target_diameter_mm: Optional[float] = 8.0,
    diameter_tolerance_mm: float = 0.05,
    holes_only: bool = True,
    angle_tol_deg: float = 2.0,
    axis_distance_tol_mm: float = 0.1,
) -> List[HoleCandidate]:
    """
    Find cylindrical holes on a STEP/B-Rep shape.

    Parameters
    ----------
    shape:
        Root shape loaded from STEP.
    target_diameter_mm:
        Desired hole diameter in millimetres. Pass None to return all holes.
    diameter_tolerance_mm:
        Absolute tolerance on diameter matching (mm).
    holes_only:
        If True, discard cylindrical bosses / outer cylinders.
    """
    raw: List[dict] = []
    for face in _iter_faces(shape):
        adaptor = BRepAdaptor_Surface(face, True)
        if adaptor.GetType() != GeomAbs_Cylinder:
            continue

        cylinder = adaptor.Cylinder()
        radius = float(cylinder.Radius())
        diameter = radius * 2.0

        if target_diameter_mm is not None:
            if abs(diameter - target_diameter_mm) > diameter_tolerance_mm:
                continue

        if holes_only and not _is_hole_cylinder(shape, face, adaptor):
            continue

        raw.append(
            {
                "radius": radius,
                "axis": cylinder.Axis(),
                "area": _face_area(face),
                "height": _cylinder_height_estimate(face, adaptor),
            }
        )

    radius_tol = diameter_tolerance_mm / 2.0
    holes = _merge_coaxial(
        raw,
        radius_tol=radius_tol,
        angle_tol_deg=angle_tol_deg,
        distance_tol=axis_distance_tol_mm,
    )
    holes.sort(key=lambda h: (-h.area_mm2, h.diameter_mm))
    return holes


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Detect cylindrical holes of a given diameter in a STEP model."
    )
    parser.add_argument("step_file", type=Path, help="Path to input .step / .stp file")
    parser.add_argument(
        "--diameter",
        type=float,
        default=8.0,
        help="Target hole diameter in mm (default: 8). Use 0 to list all holes.",
    )
    parser.add_argument(
        "--tolerance",
        type=float,
        default=0.05,
        help="Absolute diameter tolerance in mm (default: 0.05)",
    )
    parser.add_argument(
        "--include-bosses",
        action="store_true",
        help="Also report outer cylindrical faces (bosses), not only holes",
    )
    parser.add_argument(
        "--json",
        action="store_true",
        help="Print machine-readable JSON",
    )
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = _build_parser().parse_args(argv)
    target = None if args.diameter == 0 else args.diameter

    shape = load_step(args.step_file)
    holes = find_cylindrical_holes(
        shape,
        target_diameter_mm=target,
        diameter_tolerance_mm=args.tolerance,
        holes_only=not args.include_bosses,
    )

    if args.json:
        print(json.dumps([h.to_dict() for h in holes], indent=2))
    else:
        label = "all diameters" if target is None else f"{target:g} mm (±{args.tolerance:g})"
        print(f"File: {args.step_file}")
        print(f"Found {len(holes)} cylindrical hole(s) with diameter {label}:")
        for i, hole in enumerate(holes, start=1):
            axis = hole.axis_direction
            origin = hole.axis_point
            depth = f"{hole.depth_mm:.3f}" if hole.depth_mm is not None else "n/a"
            print(
                f"  [{i}] diameter={hole.diameter_mm:.4f} mm, "
                f"depth≈{depth} mm, faces={hole.face_count}, "
                f"axis=({axis[0]:.4f}, {axis[1]:.4f}, {axis[2]:.4f}), "
                f"origin=({origin[0]:.3f}, {origin[1]:.3f}, {origin[2]:.3f})"
            )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:  # noqa: BLE001 - CLI surface
        print(f"error: {exc}", file=sys.stderr)
        raise SystemExit(1)
