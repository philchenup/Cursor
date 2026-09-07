#!/usr/bin/env python3
"""V-shape groove weld path planning with Open3D visualization.

Converted from the original MATLAB 2D planner. Geometry is shown in a
right-handed frame:

    Y  weld / seam direction  (extrusion of the 2D profile)
    Z  height / plate thickness (MATLAB y)
    X  width, from the right-hand rule  X = Y × Z  (MATLAB x)

Open3D coordinate-frame colors: X red, Y green, Z blue.
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import open3d as o3d
from open3d.visualization import rendering


# ---------------------------------------------------------------------------
# Right-handed world frame
# ---------------------------------------------------------------------------
# Y: weld direction. Z: height. X: Y × Z so that X × Y = Z.
Y_AXIS = np.array([0.0, 1.0, 0.0])
Z_AXIS = np.array([0.0, 0.0, 1.0])
X_AXIS = np.cross(Y_AXIS, Z_AXIS)  # [1, 0, 0]

AXIS_COLOR = {
    "X": np.array([0.90, 0.12, 0.12]),  # red
    "Y": np.array([0.12, 0.72, 0.18]),  # green (weld)
    "Z": np.array([0.12, 0.32, 0.92]),  # blue
}

BEAD_COLOR = {
    "left": np.array([0.957, 0.635, 0.380]),
    "right": np.array([0.165, 0.616, 0.561]),
    "center": np.array([0.906, 0.435, 0.318]),
}
PLATE_COLOR = np.array([0.62, 0.68, 0.74])
LAYER_COLOR = np.array([0.30, 0.47, 0.66])
POINT_COLOR = np.array([0.10, 0.35, 0.95])
TORCH_COLOR = np.array([0.90, 0.12, 0.12])
PATH_COLOR = np.array([0.95, 0.75, 0.15])
GRID_COLOR = np.array([0.78, 0.78, 0.78])
EDGE_COLOR = np.array([0.12, 0.12, 0.12])


# ---------------------------------------------------------------------------
# Parameters (same defaults as the MATLAB script)
# ---------------------------------------------------------------------------

@dataclass
class GrooveParams:
    h: float = 16.0
    beta: float = 30.0
    g: float = 1.0
    weld_length: float = 40.0
    plate_extra: float = 8.0
    aH: float = 1.0
    v1: float = 1.0
    d: float = 1.0
    v2: float = 1.0
    n_layers_simple: int = 7
    plate_alpha: float = 0.0    # workpiece fill; 0 = wireframe only (arrows stay visible)
    bead_alpha: float = 0.45    # weld-bead slice opacity
    layer_alpha: float = 0.05   # layer-plane opacity
    arrow_scale: float = 1.8    # torch-arrow length/thickness scale
    bead_depth: float = 2.5     # bead slice thickness along Y, mm; 0 = full weld length

    @property
    def beta_rad(self) -> float:
        return math.radians(self.beta)

    @property
    def tan_beta(self) -> float:
        return math.tan(self.beta_rad)


@dataclass
class Bead:
    layer: int
    local_index: int
    kind: str
    quad_xz: np.ndarray
    weld_xz: np.ndarray
    direction_xz: np.ndarray
    sequence: int = 0

    def position_3d(self, y: float) -> np.ndarray:
        """World point (x, y, z) on the weld path."""
        return np.array([self.weld_xz[0], y, self.weld_xz[1]], dtype=float)

    def torch_dir_3d(self) -> np.ndarray:
        """Torch orientation in XZ; no Y component (perpendicular to the seam)."""
        vec = np.array([self.direction_xz[0], 0.0, self.direction_xz[1]], dtype=float)
        n = np.linalg.norm(vec)
        if n < 1e-12:
            return Z_AXIS.copy()
        return vec / n


@dataclass
class PlanResult:
    params: GrooveParams
    t: float
    S: float
    l: float
    K: int
    LN: int
    beads: list[Bead] = field(default_factory=list)
    welding_points: np.ndarray = field(default_factory=lambda: np.zeros((4, 0)))
    welding_points_3d: np.ndarray = field(default_factory=lambda: np.zeros((0, 6)))

    @property
    def point_number(self) -> int:
        return len(self.beads)


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def layer_length(m: float, t: float, tan_beta: float, g: float) -> float:
    return 2.0 * tan_beta * t * m + g


def half_width(z: float, tan_beta: float, g: float) -> float:
    return tan_beta * z + g / 2.0


def matlab_round(value: float) -> int:
    if value >= 0:
        return int(math.floor(value + 0.5))
    return int(math.ceil(value - 0.5))


def beads_in_layer(i: int, t: float, tan_beta: float, g: float) -> int:
    top = layer_length(i, t, tan_beta, g)
    bot = layer_length(i - 1, t, tan_beta, g)
    first = layer_length(1, t, tan_beta, g)
    return matlab_round((top + bot) / (first + g))


def _quad(p1, p2, p3, p4) -> np.ndarray:
    return np.array([p1, p2, p3, p4], dtype=float)


def xz_to_xyz(x: float, z: float, y: float) -> np.ndarray:
    """Map MATLAB (x, y_height) plus weld station y into the right-handed frame."""
    return np.array([x, y, z], dtype=float)


# ---------------------------------------------------------------------------
# Planning (faithful to the MATLAB algorithm)
# ---------------------------------------------------------------------------

def plan_weld(params: GrooveParams | None = None) -> PlanResult:
    p = params or GrooveParams()
    tan_b = p.tan_beta
    g = p.g

    S = p.aH * math.pi * p.v1 * (p.d ** 2) / (4.0 * p.v2)
    t = math.sqrt(S - g / tan_b) if S > g / tan_b else p.h / p.n_layers_simple
    t = p.h / p.n_layers_simple
    S = (tan_b * t + g) * t
    K = int(math.floor(p.h / t))
    l = S / t

    LN = int(math.ceil(
        (layer_length(K, t, tan_b, g) + layer_length(K - 1, t, tan_b, g))
        / (layer_length(1, t, tan_b, g) + g)
    ))

    vl = np.array([tan_b * t - l, -t])
    vr = np.array([-tan_b * t + l, -t])
    arrow = l / 2.0
    dl = arrow * np.array([
        math.sin(math.radians(45.0 - p.beta / 2.0)),
        math.cos(math.radians(45.0 - p.beta / 2.0)),
    ])
    dv = arrow * np.array([
        -math.cos(math.radians(45.0 + p.beta / 2.0)),
        math.sin(math.radians(45.0 + p.beta / 2.0)),
    ])
    up = np.array([0.0, arrow])

    beads: list[Bead] = []
    seq = 1
    y_mid = p.weld_length / 2.0
    pts_2d: list[list[float]] = []
    pts_3d: list[list[float]] = []

    def emit(layer: int, local: int, kind: str, quad, weld_xz, direction_xz) -> None:
        nonlocal seq
        weld_xz = np.asarray(weld_xz, dtype=float).reshape(2)
        direction_xz = np.asarray(direction_xz, dtype=float).reshape(2)
        beads.append(Bead(
            layer=layer,
            local_index=local,
            kind=kind,
            quad_xz=np.asarray(quad, dtype=float),
            weld_xz=weld_xz,
            direction_xz=direction_xz,
            sequence=seq,
        ))
        pts_2d.append([weld_xz[0], weld_xz[1], direction_xz[0], direction_xz[1]])
        pts_3d.append([
            weld_xz[0], y_mid, weld_xz[1],
            direction_xz[0], 0.0, direction_xz[1],
        ])
        seq += 1

    for i in range(1, K + 1):
        z_top = i * t
        z_bot = (i - 1) * t
        xr = half_width(z_top, tan_b, g)
        xl = -xr
        xr_b = half_width(z_bot, tan_b, g)
        xl_b = -xr_b
        N = beads_in_layer(i, t, tan_b, g)

        if N == 1:
            quad = _quad((xl, z_top), (xr, z_top), (xr_b, z_bot), (xl_b, z_bot))
            emit(i, 1, "center", quad, (0.0, 0.0), up)
            continue

        lp = N // 2
        left_welds: list[np.ndarray] = []
        left_quads: list[np.ndarray] = []
        for j in range(1, lp + 1):
            x = xl + j * l
            weld = np.array([x + vl[0], z_top + vl[1]])
            quad = _quad(
                (xl + (j - 1) * l, z_top),
                (xl + j * l, z_top),
                (xl_b + j * l, z_bot),
                (xl_b + (j - 1) * l, z_bot),
            )
            left_welds.append(weld)
            left_quads.append(quad)

        right_welds: list[np.ndarray] = []
        right_quads: list[np.ndarray] = []
        k = N - 1
        for j in range(lp + 1, N):
            x = xr - (N - k) * l
            weld = np.array([x + vr[0], z_top + vr[1]])
            n_from_right = N - k
            quad = _quad(
                (xr - n_from_right * l, z_top),
                (xr - (n_from_right - 1) * l, z_top),
                (xr_b - (n_from_right - 1) * l, z_bot),
                (xr_b - n_from_right * l, z_bot),
            )
            right_welds.append(weld)
            right_quads.append(quad)
            k -= 1

        if N == 2:
            emit(i, 1, "left", left_quads[0], left_welds[0], dl)
            weld2 = np.array([l / 2.0, left_welds[0][1]])
            quad_c = _quad(
                (xl + l, z_top), (xr, z_top), (xr_b, z_bot), (xl_b + l, z_bot),
            )
            emit(i, 2, "center", quad_c, weld2, up)
            continue

        for j, (weld, quad) in enumerate(zip(left_welds, left_quads), start=1):
            emit(i, j, "left", quad, weld, dl)
        for j, (weld, quad) in enumerate(zip(right_welds, right_quads), start=lp + 1):
            emit(i, j, "right", quad, weld, dv)

        inner_left_top = xl + lp * l
        inner_left_bot = xl_b + lp * l
        if right_quads:
            inner_right_top = xr - (N - 1 - lp) * l
            inner_right_bot = xr_b - (N - 1 - lp) * l
        else:
            inner_right_top, inner_right_bot = xr, xr_b
        quad_c = _quad(
            (inner_left_top, z_top),
            (inner_right_top, z_top),
            (inner_right_bot, z_bot),
            (inner_left_bot, z_bot),
        )
        emit(i, N, "center", quad_c, 0.5 * (left_welds[-1] + right_welds[-1]), up)

    welding_points = np.array(pts_2d, dtype=float).T if pts_2d else np.zeros((4, 0))
    welding_points_3d = np.array(pts_3d, dtype=float) if pts_3d else np.zeros((0, 6))
    return PlanResult(
        params=p, t=t, S=S, l=l, K=K, LN=LN, beads=beads,
        welding_points=welding_points, welding_points_3d=welding_points_3d,
    )


# ---------------------------------------------------------------------------
# Open3D mesh / line helpers
# ---------------------------------------------------------------------------

def _paint(mesh: o3d.geometry.TriangleMesh, color: np.ndarray) -> o3d.geometry.TriangleMesh:
    mesh.compute_vertex_normals()
    mesh.paint_uniform_color(color.tolist())
    return mesh


def _mesh_from_vertices_triangles(vertices, triangles, color) -> o3d.geometry.TriangleMesh:
    mesh = o3d.geometry.TriangleMesh()
    mesh.vertices = o3d.utility.Vector3dVector(np.asarray(vertices, dtype=float))
    mesh.triangles = o3d.utility.Vector3iVector(np.asarray(triangles, dtype=np.int32))
    mesh.remove_duplicated_triangles()
    mesh.remove_degenerate_triangles()
    return _paint(mesh, color)


def _lineset(points, lines, color, extra_colors=None) -> o3d.geometry.LineSet:
    ls = o3d.geometry.LineSet()
    ls.points = o3d.utility.Vector3dVector(np.asarray(points, dtype=float))
    ls.lines = o3d.utility.Vector2iVector(np.asarray(lines, dtype=np.int32))
    if extra_colors is not None:
        ls.colors = o3d.utility.Vector3dVector(np.asarray(extra_colors, dtype=float))
    else:
        ls.paint_uniform_color(np.asarray(color, dtype=float).tolist())
    return ls


def _rotation_from_z(direction: np.ndarray) -> np.ndarray:
    """Rotation that maps +Z (Open3D arrow default) onto `direction`."""
    z = np.array([0.0, 0.0, 1.0])
    v = np.asarray(direction, dtype=float)
    n = np.linalg.norm(v)
    if n < 1e-12:
        return np.eye(3)
    v = v / n
    axis = np.cross(z, v)
    axis_n = np.linalg.norm(axis)
    if axis_n < 1e-12:
        if np.dot(z, v) > 0:
            return np.eye(3)
        return o3d.geometry.get_rotation_matrix_from_axis_angle(np.array([math.pi, 0.0, 0.0]))
    angle = math.acos(float(np.clip(np.dot(z, v), -1.0, 1.0)))
    return o3d.geometry.get_rotation_matrix_from_axis_angle(axis / axis_n * angle)


def make_arrow(origin, direction, color, cylinder_r=0.18, cone_r=0.38) -> o3d.geometry.TriangleMesh:
    vec = np.asarray(direction, dtype=float)
    length = float(np.linalg.norm(vec))
    if length < 1e-9:
        return o3d.geometry.TriangleMesh()
    cone_h = min(0.32 * length, 2.2)
    cyl_h = max(length - cone_h, 0.05 * length)
    arrow = o3d.geometry.TriangleMesh.create_arrow(
        cylinder_radius=cylinder_r,
        cone_radius=cone_r,
        cylinder_height=cyl_h,
        cone_height=cone_h,
        resolution=16,
        cylinder_split=1,
        cone_split=1,
    )
    arrow.rotate(_rotation_from_z(vec), center=(0.0, 0.0, 0.0))
    arrow.translate(np.asarray(origin, dtype=float))
    return _paint(arrow, color)


def make_sphere(center, radius, color) -> o3d.geometry.TriangleMesh:
    sph = o3d.geometry.TriangleMesh.create_sphere(radius=radius, resolution=12)
    sph.translate(np.asarray(center, dtype=float))
    return _paint(sph, color)


def _extrude_quad_mesh(quad_xz: np.ndarray, y0: float, y1: float, color) -> o3d.geometry.TriangleMesh:
    q = np.asarray(quad_xz, dtype=float)
    front = np.column_stack([q[:, 0], np.full(4, y0), q[:, 1]])
    back = np.column_stack([q[:, 0], np.full(4, y1), q[:, 1]])
    v = np.vstack([front, back])
    tris = [
        [0, 1, 2], [0, 2, 3],
        [4, 6, 5], [4, 7, 6],
        [0, 4, 5], [0, 5, 1],
        [1, 5, 6], [1, 6, 2],
        [2, 6, 7], [2, 7, 3],
        [3, 7, 4], [3, 4, 0],
    ]
    return _mesh_from_vertices_triangles(v, tris, color)


# ---------------------------------------------------------------------------
# Coordinate frame (Open3D, right-handed, Y = weld)
# ---------------------------------------------------------------------------

def _letter_lines(letter: str, origin: np.ndarray, axis: str, scale: float):
    """Simple 3D stroke letters sitting at an axis tip."""
    s = scale
    pts2 = {
        "X": [(-s, -s), (s, s), (-s, s), (s, -s)],
        "Y": [(0, -s), (0, 0), (-s, s), (s, s), (0, 0)],
        "Z": [(-s, s), (s, s), (-s, -s), (s, -s)],
    }[letter]
    segs = {
        "X": [(0, 1), (2, 3)],
        "Y": [(0, 1), (1, 2), (1, 3)],
        "Z": [(0, 1), (1, 2), (2, 3)],
    }[letter]
    points = []
    for a, b in pts2:
        if axis == "X":
            points.append(origin + np.array([0.0, a, b]))
        elif axis == "Y":
            points.append(origin + np.array([a, 0.0, b]))
        else:
            points.append(origin + np.array([a, b, 0.0]))
    return np.array(points), segs


def make_coordinate_frame(size: float = 12.0, origin=None) -> list:
    """Visible world frame: X red, Y green (weld), Z blue.

    Placed slightly in -Y so the triad sits in front of the groove and all
    three axes stay readable. Open3D convention: X red, Y green, Z blue.
    """
    origin = np.array([0.0, -6.0, 0.0]) if origin is None else np.asarray(origin, dtype=float)
    geoms: list = []
    frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=size, origin=origin)
    geoms.append(frame)

    letter_scale = size * 0.14
    letter_pts: list[np.ndarray] = []
    letter_lines: list[tuple[int, int]] = []
    letter_colors: list[np.ndarray] = []
    placements = [
        ("X", origin + X_AXIS * (size + 2.4), "X"),
        ("Y", origin + Y_AXIS * (size + 2.4), "Y"),
        ("Z", origin + Z_AXIS * (size + 2.4), "Z"),
    ]
    for letter, tip, axis_name in placements:
        pts, segs = _letter_lines(letter, tip, axis_name, letter_scale)
        base = len(letter_pts)
        letter_pts.extend(pts)
        for a, b in segs:
            letter_lines.append((base + a, base + b))
            letter_colors.append(AXIS_COLOR[letter])
    geoms.append(_lineset(letter_pts, letter_lines, AXIS_COLOR["X"], extra_colors=letter_colors))
    return geoms


def make_weld_axis_arrow(p: GrooveParams) -> o3d.geometry.TriangleMesh:
    """Long green arrow beside the plates, parallel to +Y (weld direction)."""
    x_left = -(p.h * p.tan_beta + p.g / 2.0 + p.plate_extra + 5.0)
    start = np.array([x_left, 0.0, 0.0])
    return make_arrow(start, Y_AXIS * p.weld_length, AXIS_COLOR["Y"],
                      cylinder_r=0.35, cone_r=0.75)


def make_xy_grid(p: GrooveParams, step: float = 5.0) -> o3d.geometry.LineSet:
    """Ground grid on Z = 0 (XY plane) so X and weld-Y are readable."""
    x_span = p.h * p.tan_beta + p.g / 2.0 + p.plate_extra + 8.0
    y0, y1 = -8.0, p.weld_length
    xs = np.arange(-math.floor(x_span / step) * step, x_span + 0.5 * step, step)
    ys = np.arange(y0, y1 + 0.5 * step, step)
    points, lines = [], []
    for y in ys:
        i0 = len(points)
        points.append([-x_span, y, 0.0])
        points.append([x_span, y, 0.0])
        lines.append([i0, i0 + 1])
    for x in xs:
        i0 = len(points)
        points.append([x, y0, 0.0])
        points.append([x, y1, 0.0])
        lines.append([i0, i0 + 1])
    return _lineset(points, lines, GRID_COLOR)


# ---------------------------------------------------------------------------
# Scene contents
# ---------------------------------------------------------------------------

def make_plate_meshes(p: GrooveParams) -> list[o3d.geometry.TriangleMesh]:
    tan_b = p.tan_beta
    y0, y1 = 0.0, p.weld_length
    x_in_top = p.h * tan_b + p.g / 2.0
    x_out = x_in_top + p.plate_extra
    meshes = []
    for sign in (-1.0, 1.0):
        v = np.array([
            [sign * x_out, y0, 0.0],
            [sign * p.g / 2.0, y0, 0.0],
            [sign * x_in_top, y0, p.h],
            [sign * x_out, y0, p.h],
            [sign * x_out, y1, 0.0],
            [sign * p.g / 2.0, y1, 0.0],
            [sign * x_in_top, y1, p.h],
            [sign * x_out, y1, p.h],
        ])
        tris = [
            [1, 0, 3], [1, 3, 2],
            [4, 5, 6], [4, 6, 7],
            [0, 4, 7], [0, 7, 3],
            [1, 2, 6], [1, 6, 5],
            [3, 7, 6], [3, 6, 2],
            [0, 1, 5], [0, 5, 4],
        ]
        if sign < 0:
            tris = [[a, c, b] for a, b, c in tris]
        meshes.append(_mesh_from_vertices_triangles(v, tris, PLATE_COLOR))
    return meshes


def make_groove_edges(p: GrooveParams) -> o3d.geometry.LineSet:
    tan_b = p.tan_beta
    A = xz_to_xyz(-p.g / 2.0, 0.0, 0.0)
    B = xz_to_xyz(p.g / 2.0, 0.0, 0.0)
    C = xz_to_xyz(p.h * tan_b + p.g / 2.0, p.h, 0.0)
    D = xz_to_xyz(-p.h * tan_b - p.g / 2.0, p.h, 0.0)
    off = np.array([0.0, p.weld_length, 0.0])
    A2, B2, C2, D2 = A + off, B + off, C + off, D + off
    pts = [A, B, C, D, A2, B2, C2, D2]
    lines = [
        [0, 1], [0, 3], [1, 2],
        [4, 5], [4, 7], [5, 6],
        [0, 4], [1, 5], [2, 6], [3, 7],
        [3, 2], [7, 6],
    ]
    return _lineset(pts, lines, EDGE_COLOR)


def make_layer_planes(plan: PlanResult) -> list[o3d.geometry.TriangleMesh]:
    p = plan.params
    meshes = []
    for i in range(1, plan.K + 1):
        z = i * plan.t
        x = half_width(z, p.tan_beta, p.g)
        v = np.array([
            [-x, 0.0, z], [x, 0.0, z], [x, p.weld_length, z], [-x, p.weld_length, z],
        ])
        tris = [[0, 1, 2], [0, 2, 3]]
        meshes.append(_mesh_from_vertices_triangles(v, tris, LAYER_COLOR))
    return meshes


def make_segment_dividers(plan: PlanResult) -> o3d.geometry.LineSet:
    p = plan.params
    tan_b, t, l, g = p.tan_beta, plan.t, plan.l, p.g
    vl = np.array([tan_b * t, -t])
    vr = np.array([-tan_b * t, -t])
    points, lines = [], []

    def add_seg(a, b):
        i0 = len(points)
        points.extend([a, b])
        lines.append([i0, i0 + 1])

    for i in range(1, plan.K + 1):
        z = i * t
        xr = half_width(z, tan_b, g)
        xl = -xr
        N = beads_in_layer(i, t, tan_b, g)
        if N <= 1:
            continue
        lp = N // 2
        for j in range(1, lp + 1):
            x = xl + j * l
            a = xz_to_xyz(x, z, 0.0)
            b = xz_to_xyz(x + vl[0], z + vl[1], 0.0)
            a2 = xz_to_xyz(x, z, p.weld_length)
            b2 = xz_to_xyz(x + vl[0], z + vl[1], p.weld_length)
            add_seg(a, b)
            add_seg(a2, b2)
            add_seg(a, a2)
            add_seg(b, b2)
        for j in range(lp + 1, N):
            x = xr - (N - j) * l
            a = xz_to_xyz(x, z, 0.0)
            b = xz_to_xyz(x + vr[0], z + vr[1], 0.0)
            a2 = xz_to_xyz(x, z, p.weld_length)
            b2 = xz_to_xyz(x + vr[0], z + vr[1], p.weld_length)
            add_seg(a, b)
            add_seg(a2, b2)
            add_seg(a, a2)
            add_seg(b, b2)
    if not points:
        return o3d.geometry.LineSet()
    return _lineset(points, lines, EDGE_COLOR)


def make_bead_meshes(plan: PlanResult) -> list[o3d.geometry.TriangleMesh]:
    L = plan.params.weld_length
    depth = plan.params.bead_depth
    if depth <= 0:
        y0, y1 = 0.0, L
    else:
        y_mid = L / 2.0
        half = 0.5 * depth
        y0, y1 = y_mid - half, y_mid + half
    return [
        _extrude_quad_mesh(bead.quad_xz, y0, y1, BEAD_COLOR[bead.kind])
        for bead in plan.beads
    ]


def make_weld_paths(plan: PlanResult) -> o3d.geometry.LineSet:
    L = plan.params.weld_length
    points, lines, colors = [], [], []
    for bead in plan.beads:
        i0 = len(points)
        points.append(bead.position_3d(0.0))
        points.append(bead.position_3d(L))
        lines.append([i0, i0 + 1])
        colors.append(BEAD_COLOR[bead.kind])
    return _lineset(points, lines, PATH_COLOR, extra_colors=colors)


def make_weld_poses(plan: PlanResult) -> list[o3d.geometry.TriangleMesh]:
    y = plan.params.weld_length / 2.0
    scale = max(plan.params.arrow_scale, 0.5)
    arrow_len = max(plan.t * 2.6, 5.5) * scale
    geoms: list[o3d.geometry.TriangleMesh] = []
    for bead in plan.beads:
        origin = bead.position_3d(y)
        geoms.append(make_sphere(origin, radius=0.22, color=POINT_COLOR))
        geoms.append(make_arrow(
            origin, bead.torch_dir_3d() * arrow_len, TORCH_COLOR,
            cylinder_r=0.16 * scale, cone_r=0.38 * scale,
        ))
    return geoms


def sequence_labels(plan: PlanResult) -> list[tuple[np.ndarray, str]]:
    y = plan.params.weld_length * 0.92
    dz = 0.25 * plan.t
    return [(bead.position_3d(y) + np.array([0.0, 0.0, dz]), str(bead.sequence))
            for bead in plan.beads]


def build_scene(plan: PlanResult) -> list:
    """Full Open3D scene: frame, plates, beads, paths, torch poses."""
    p = plan.params
    geoms: list = []
    geoms.append(make_xy_grid(p))
    geoms.extend(make_coordinate_frame(size=max(12.0, p.h * 0.85)))
    geoms.append(make_weld_axis_arrow(p))
    geoms.extend(make_plate_meshes(p))
    geoms.append(make_groove_edges(p))
    geoms.extend(make_layer_planes(plan))
    geoms.extend(make_bead_meshes(plan))
    geoms.append(make_segment_dividers(plan))
    geoms.append(make_weld_paths(plan))
    geoms.extend(make_weld_poses(plan))
    return [g for g in geoms if g is not None]


# ---------------------------------------------------------------------------
# Camera: Z-up, look along +Y (weld), X to the right (right-hand)
# ---------------------------------------------------------------------------

def _camera_eye_lookat_up(plan: PlanResult, view: str) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    p = plan.params
    lookat = np.array([0.0, p.weld_length * 0.35, p.h * 0.40])
    up = Z_AXIS
    span = max(p.weld_length, p.h * 2.0, 30.0)
    if view == "end":
        # Camera on -Y, looking toward +Y: X right, Z up (right-hand).
        eye = lookat + np.array([0.0, -1.85 * span, 0.05 * span])
    else:
        # 3/4 view from -Y / +X so the green weld axis recedes into the scene.
        eye = lookat + np.array([0.85 * span, -1.45 * span, 0.50 * span])
    return eye, lookat, up


def _vertex_color_material(alpha: float = 1.0) -> rendering.MaterialRecord:
    """Keep TriangleMesh vertex colors. alpha < 1 uses transparent shader."""
    mat = rendering.MaterialRecord()
    a = float(np.clip(alpha, 0.0, 1.0))
    mat.shader = "defaultLit" if a >= 0.999 else "defaultLitTransparency"
    mat.base_color = [1.0, 1.0, 1.0, a]
    mat.base_roughness = 0.45
    mat.base_metallic = 0.0
    return mat


def _unlit_material() -> rendering.MaterialRecord:
    """Opaque unlit material so pose/axis arrows stay bright."""
    mat = rendering.MaterialRecord()
    mat.shader = "defaultUnlit"
    mat.base_color = [1.0, 1.0, 1.0, 1.0]
    return mat


def _line_material(width: float = 2.0) -> rendering.MaterialRecord:
    mat = rendering.MaterialRecord()
    mat.shader = "unlitLine"
    mat.line_width = width
    mat.base_color = [1.0, 1.0, 1.0, 1.0]
    return mat


def scene_items(plan: PlanResult) -> list[tuple[str, object, rendering.MaterialRecord]]:
    """Named geometries with materials. Opaque arrows are last so they stay visible."""
    p = plan.params
    items: list[tuple[str, object, rendering.MaterialRecord]] = [
        ("grid", make_xy_grid(p), _line_material(1.0)),
        ("edges", make_groove_edges(p), _line_material(2.8)),
        ("dividers", make_segment_dividers(plan), _line_material(1.6)),
        ("paths", make_weld_paths(plan), _line_material(3.2)),
    ]
    plate_mat = _vertex_color_material(p.plate_alpha)
    layer_mat = _vertex_color_material(p.layer_alpha)
    bead_mat = _vertex_color_material(p.bead_alpha)
    arrow_mat = _unlit_material()

    if p.plate_alpha > 0.01:
        for i, mesh in enumerate(make_plate_meshes(p)):
            items.append((f"plate_{i}", mesh, plate_mat))
    if p.layer_alpha > 0.01:
        for i, mesh in enumerate(make_layer_planes(plan)):
            items.append((f"layer_{i}", mesh, layer_mat))
    if p.bead_alpha > 0.01:
        for i, mesh in enumerate(make_bead_meshes(plan)):
            items.append((f"bead_{i}", mesh, bead_mat))

    axis_size = max(12.0, p.h * 0.85)
    for i, geom in enumerate(make_coordinate_frame(size=axis_size)):
        if isinstance(geom, o3d.geometry.LineSet):
            items.append((f"axis_letter_{i}", geom, _line_material(6.0)))
        else:
            items.append((f"axis_{i}", geom, arrow_mat))
    items.append(("weld_y_arrow", make_weld_axis_arrow(p), arrow_mat))
    for i, mesh in enumerate(make_weld_poses(plan)):
        items.append((f"pose_{i}", mesh, arrow_mat))
    return items


def render_screenshot(plan: PlanResult, path: Path, view: str = "perspective",
                      width: int = 1600, height: int = 1000) -> None:
    """Offscreen Filament render, Z-up right-handed camera."""
    renderer = rendering.OffscreenRenderer(width, height)
    scene = renderer.scene
    scene.set_background([1.0, 1.0, 1.0, 1.0])
    scene.scene.set_sun_light([0.35, -0.55, -0.85], [1.0, 1.0, 1.0], 90_000)
    scene.scene.enable_sun_light(True)

    for name, geom, mat in scene_items(plan):
        scene.add_geometry(name, geom, mat)

    eye, lookat, up = _camera_eye_lookat_up(plan, view)
    scene.camera.look_at(lookat, eye, up)
    img = renderer.render_to_image()
    o3d.io.write_image(str(path), img)


def show_open3d(plan: PlanResult, view: str = "perspective") -> None:
    """Interactive Open3D window; uses Filament materials so alpha is applied."""
    app = o3d.visualization.gui.Application.instance
    app.initialize()
    vis = o3d.visualization.O3DVisualizer("V-groove weld  |  Y weld  |  X = Y × Z", 1600, 1000)
    vis.show_skybox(False)
    vis.show_axes = True
    vis.point_size = 6
    vis.line_width = 3
    for name, geom, mat in scene_items(plan):
        vis.add_geometry(name, geom, mat)
    eye, lookat, up = _camera_eye_lookat_up(plan, view)
    vis.setup_camera(60.0, lookat, eye, up)
    app.add_window(vis)
    app.run()


def show_open3d_with_labels(plan: PlanResult) -> None:
    """GUI visualizer: coordinate-frame colors + sequence labels at weld points."""
    app = o3d.visualization.gui.Application.instance
    app.initialize()
    vis = o3d.visualization.O3DVisualizer("V-groove weld  |  Y weld  |  X = Y × Z", 1600, 1000)
    vis.show_skybox(False)
    vis.show_axes = True  # Open3D RGB triad: X red, Y green, Z blue
    try:
        vis.show_ground = True
        vis.ground_plane = o3d.visualization.O3DVisualizer.GroundPlane.XY
    except Exception:
        pass
    vis.point_size = 6
    vis.line_width = 3

    for name, geom, mat in scene_items(plan):
        vis.add_geometry(name, geom, mat)

    frame_origin = np.array([0.0, -6.0, 0.0])
    vis.add_3d_label((frame_origin + X_AXIS * 14.0).tolist(), "X  (Y×Z)")
    vis.add_3d_label((frame_origin + Y_AXIS * 16.0).tolist(), "Y  weld")
    vis.add_3d_label((frame_origin + Z_AXIS * 14.0).tolist(), "Z  height")
    for pos, text in sequence_labels(plan):
        vis.add_3d_label(pos.tolist(), text)

    eye, lookat, up = _camera_eye_lookat_up(plan, "perspective")
    vis.setup_camera(60.0, lookat, eye, up)
    app.add_window(vis)
    app.run()


def parameter_text(plan: PlanResult) -> str:
    p = plan.params
    return (
        "V-shape groove parameter\n"
        f"  Thickness: {p.h:g} mm\n"
        f"  Angle: {p.beta:g} deg  (included  {2 * p.beta:g})\n"
        f"  Assembly clearance: {p.g:g} mm\n"
        f"  Bead thickness: {plan.t:.4g} mm\n"
        f"  Number of layer: {plan.K}\n"
        f"  Number of weld points: {plan.point_number}\n"
        f"  Weld length Y: {p.weld_length:g} mm\n"
        f"  Alpha  plate/bead/layer: {p.plate_alpha:g}/{p.bead_alpha:g}/{p.layer_alpha:g}\n"
        f"  Bead depth (Y slice): {p.bead_depth:g} mm\n"
        "Frame (right-hand):  X = Y × Z,  Y = weld,  Z = height"
    )


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--h", dest="h", type=float, default=16.0)
    parser.add_argument("--beta", type=float, default=30.0)
    parser.add_argument("--g", dest="g", type=float, default=1.0)
    parser.add_argument("--weld-length", type=float, default=40.0)
    parser.add_argument(
        "--plate-alpha", type=float, default=0.0,
        help="workpiece fill opacity in [0, 1]; 0 = wireframe only so arrows stay visible",
    )
    parser.add_argument(
        "--bead-alpha", type=float, default=0.45,
        help="weld-bead opacity in [0, 1]; lower makes torch arrows easier to see",
    )
    parser.add_argument(
        "--layer-alpha", type=float, default=0.05,
        help="layer-plane opacity in [0, 1]",
    )
    parser.add_argument(
        "--bead-depth", type=float, default=2.5,
        help="bead slice thickness along Y in mm; 0 extrudes the full weld length",
    )
    parser.add_argument(
        "--arrow-scale", type=float, default=1.8,
        help="scale of torch pose arrows (length and thickness)",
    )
    parser.add_argument("--outdir", type=Path, default=Path("figures"))
    parser.add_argument("--show", action="store_true", help="open interactive Open3D window")
    parser.add_argument("--gui", action="store_true", help="Open3D GUI with 3D text labels")
    parser.add_argument("--view", choices=["perspective", "end"], default="perspective")
    parser.add_argument("--no-save", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    params = GrooveParams(
        h=args.h, beta=args.beta, g=args.g, weld_length=args.weld_length,
        plate_alpha=float(np.clip(args.plate_alpha, 0.0, 1.0)),
        bead_alpha=float(np.clip(args.bead_alpha, 0.0, 1.0)),
        layer_alpha=float(np.clip(args.layer_alpha, 0.0, 1.0)),
        arrow_scale=max(args.arrow_scale, 0.1),
        bead_depth=max(args.bead_depth, 0.0),
    )
    plan = plan_weld(params)

    print("V-shape groove weld planning  (Open3D, right-handed)")
    print("  X = Y × Z  |  Y = weld direction  |  Z = height")
    print(f"  X_AXIS = {X_AXIS}, Y_AXIS = {Y_AXIS}, Z_AXIS = {Z_AXIS}")
    print(f"  X·(Y×Z) = {np.dot(X_AXIS, np.cross(Y_AXIS, Z_AXIS)):.1f}  (should be 1)")
    print(parameter_text(plan))
    print(f"  parallelogram width l = {plan.l:.4f} mm, max beads LN = {plan.LN}")
    np.set_printoptions(precision=4, suppress=True)
    print("  welding_points_3d (N x 6) [x, y, z, dx, dy, dz]:")
    print(plan.welding_points_3d)

    if not args.no_save:
        args.outdir.mkdir(parents=True, exist_ok=True)
        persp = args.outdir / "vgroove_open3d_perspective.png"
        endv = args.outdir / "vgroove_open3d_endview.png"
        render_screenshot(plan, persp, view="perspective")
        render_screenshot(plan, endv, view="end")
        print(f"  saved {persp}")
        print(f"  saved {endv}")

    if args.gui:
        show_open3d_with_labels(plan)
    elif args.show:
        show_open3d(plan, view=args.view)


if __name__ == "__main__":
    main()
