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
    _, _, height = _axis_endpoints_from_face(adaptor)
    return height


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


def _axis_endpoints_from_face(adaptor: BRepAdaptor_Surface) -> Tuple[gp_Pnt, gp_Pnt, float]:
    """
    Project the cylindrical face V-ends onto its axis.

    ``gp_Cylinder.Location()`` is only a geometric definition point and often
    sits at the bottom parameter end; face UV bounds give the real hole extent.
    """
    axis = adaptor.Cylinder().Axis()
    u_mid = 0.5 * (adaptor.FirstUParameter() + adaptor.LastUParameter())
    p0 = adaptor.Value(u_mid, adaptor.FirstVParameter())
    p1 = adaptor.Value(u_mid, adaptor.LastVParameter())
    a0 = _project_point_to_axis(p0, axis)
    a1 = _project_point_to_axis(p1, axis)
    return a0, a1, a0.Distance(a1)


def _prefer_top_axis_point(a0: gp_Pnt, a1: gp_Pnt) -> gp_Pnt:
    """Pick the hole opening with larger world-Z (top side)."""
    return a0 if a0.Z() >= a1.Z() else a1


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

        # Collect both axial ends from all faces in the cluster, then take the top.
        ends: List[gp_Pnt] = []
        for c in cluster:
            ends.extend([c["axis_end0"], c["axis_end1"]])
        top = max(ends, key=lambda p: p.Z())

        direction = ref["axis"].Direction()
        # Point axis "outward" toward the top opening when possible.
        bottom = min(ends, key=lambda p: p.Z())
        up = gp_Vec(bottom, top)
        if up.Magnitude() > 1e-12 and up.Dot(gp_Vec(direction.XYZ())) < 0:
            direction = direction.Reversed()

        holes.append(
            HoleCandidate(
                diameter_mm=radius * 2.0,
                radius_mm=radius,
                axis_point=_point(top),
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

        end0, end1, height = _axis_endpoints_from_face(adaptor)
        raw.append(
            {
                "radius": radius,
                "axis": cylinder.Axis(),
                "area": _face_area(face),
                "height": height,
                "axis_end0": end0,
                "axis_end1": end1,
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


def _orthonormal_xy(axis_direction: Sequence[float]) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
    """Build a right-handed X/Y basis for a Z axis along ``axis_direction``."""
    z = gp_Vec(float(axis_direction[0]), float(axis_direction[1]), float(axis_direction[2]))
    if z.Magnitude() < 1e-12:
        raise ValueError("hole axis direction is degenerate")
    z.Normalize()

    # Prefer a stable reference that is not parallel to Z.
    ref = gp_Vec(0.0, 0.0, 1.0) if abs(z.Z()) < 0.9 else gp_Vec(1.0, 0.0, 0.0)
    x = z.Crossed(ref)
    if x.Magnitude() < 1e-12:
        ref = gp_Vec(0.0, 1.0, 0.0)
        x = z.Crossed(ref)
    x.Normalize()
    y = z.Crossed(x)
    y.Normalize()
    return _unit(x), _unit(y)


def hole_frame_location(hole: HoleCandidate):
    """
    Build a CadQuery ``Location`` for a hole frame.

    Convention: origin = hole axis point, Z = hole axis, X/Y = orthonormal plane.
    """
    from cadquery import Location, Plane

    x_dir, _y_dir = _orthonormal_xy(hole.axis_direction)
    plane = Plane(origin=hole.axis_point, xDir=x_dir, normal=hole.axis_direction)
    return Location(plane)


def model_frame_location(origin: Tuple[float, float, float] = (0.0, 0.0, 0.0)):
    """CadQuery ``Location`` for the model/world XYZ frame at ``origin``."""
    from cadquery import Location, Plane

    return Location(Plane(origin=origin, xDir=(1.0, 0.0, 0.0), normal=(0.0, 0.0, 1.0)))


def show_model_and_hole_frames(
    shape: TopoDS_Shape,
    holes: Sequence[HoleCandidate],
    *,
    show_model_frame: bool = True,
    model_frame_origin: Tuple[float, float, float] = (0.0, 0.0, 0.0),
    axis_scale: Optional[float] = None,
    alpha: float = 0.65,
    title: str = "STEP model + hole frames",
    screenshot: Optional[str | Path] = None,
    interact: Optional[bool] = None,
) -> None:
    """
    Display the STEP solid together with RGB XYZ frames for the model and each hole.

    - Red = X, Green = Y, Blue = Z (hole Z aligns with the cylinder axis)
    - Pass ``screenshot`` to save a PNG; set ``interact=False`` for headless use
    """
    from cadquery.vis import show

    frames = []
    if show_model_frame:
        frames.append(model_frame_location(model_frame_origin))
    frames.extend(hole_frame_location(h) for h in holes)

    if axis_scale is None:
        # Scale triads from the largest hole diameter / depth so axes stay readable.
        sizes = [h.diameter_mm for h in holes] + [h.depth_mm or 0.0 for h in holes] + [10.0]
        axis_scale = max(sizes) * 0.75

    if interact is None:
        interact = screenshot is None

    show(
        shape,
        *frames,
        scale=float(axis_scale),
        alpha=alpha,
        edges=True,
        title=title,
        screenshot=str(screenshot) if screenshot is not None else None,
        interact=interact,
        trihedron=True,
    )


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
    parser.add_argument(
        "--show",
        action="store_true",
        help="Display the model with model/hole XYZ coordinate frames",
    )
    parser.add_argument(
        "--screenshot",
        type=Path,
        default=None,
        help="Save a PNG of the visualization (implies --show; use with --no-interact for headless)",
    )
    parser.add_argument(
        "--no-interact",
        action="store_true",
        help="Do not open an interactive window (useful with --screenshot)",
    )
    parser.add_argument(
        "--axis-scale",
        type=float,
        default=None,
        help="Length scale of displayed XYZ triads (mm). Default: auto from hole size.",
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

    if args.show or args.screenshot is not None:
        show_model_and_hole_frames(
            shape,
            holes,
            axis_scale=args.axis_scale,
            screenshot=args.screenshot,
            interact=not args.no_interact,
        )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:  # noqa: BLE001 - CLI surface
        print(f"error: {exc}", file=sys.stderr)
        raise SystemExit(1)
