#!/usr/bin/env python3
"""V-shape groove weld path planning with 3D visualization.

Converted from the original MATLAB 2D planner. The groove cross-section
lives in the X–Z plane (X = width, Z = height). The weld is extruded along
Y (groove length) so layers, bead segments, sequence numbers and torch
poses can be shown in 3D.

Coordinate mapping from MATLAB:
    MATLAB x  ->  Python x   (groove width)
    MATLAB y  ->  Python z   (groove height / plate thickness)
    (new)     ->  Python y   (weld / extrusion length)
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass, field
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d.art3d import Line3DCollection, Poly3DCollection


# ---------------------------------------------------------------------------
# Parameters (same defaults as the MATLAB script)
# ---------------------------------------------------------------------------

@dataclass
class GrooveParams:
    h: float = 16.0          # plate thickness / groove height, mm
    beta: float = 30.0       # bevel angle, deg; total included angle is 2*beta
    g: float = 1.0           # root opening (assembly clearance), mm
    weld_length: float = 40.0  # extrusion length along Y, mm (3D only)
    plate_extra: float = 8.0   # extra plate width outside the groove, mm

    # welding parameter information (kept for compatibility with MATLAB)
    aH: float = 1.0          # deposition coefficient
    v1: float = 1.0          # wire feed rate
    d: float = 1.0           # wire diameter
    v2: float = 1.0          # welding speed
    n_layers_simple: int = 7  # MATLAB overrides t = h/7 for simplicity

    @property
    def beta_rad(self) -> float:
        return math.radians(self.beta)

    @property
    def tan_beta(self) -> float:
        return math.tan(self.beta_rad)


@dataclass
class Bead:
    """One weld bead (parallelogram or center trapezoid) in a layer."""

    layer: int
    local_index: int
    kind: str  # 'left', 'right', 'center'
    quad_xz: np.ndarray          # (4, 2) cyclic cross-section vertices
    weld_xz: np.ndarray          # (2,) MATLAB weld-point (x, z)
    direction_xz: np.ndarray     # (2,) torch direction in the X–Z plane
    sequence: int = 0


@dataclass
class PlanResult:
    params: GrooveParams
    t: float                     # layer thickness, mm
    S: float                     # bead cross-section area, mm^2
    l: float                     # parallelogram bead width, mm
    K: int                       # number of layers
    LN: int                      # max beads in any layer
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
    """Width of the groove at the top of layer m (MATLAB L(m)). m may be 0."""
    return 2.0 * tan_beta * t * m + g


def half_width(z: float, tan_beta: float, g: float) -> float:
    """Groove half-width at height z (MATLAB right_x)."""
    return tan_beta * z + g / 2.0


def matlab_round(value: float) -> int:
    """Round half away from zero, matching MATLAB round() for real scalars."""
    if value >= 0:
        return int(math.floor(value + 0.5))
    return int(math.ceil(value - 0.5))


def beads_in_layer(i: int, t: float, tan_beta: float, g: float) -> int:
    """MATLAB: N = round((L(i)+L(i-1))/(L(1)+g)), 1-based layer index i."""
    top = layer_length(i, t, tan_beta, g)
    bot = layer_length(i - 1, t, tan_beta, g)
    first = layer_length(1, t, tan_beta, g)
    return matlab_round((top + bot) / (first + g))


def _quad(p1, p2, p3, p4) -> np.ndarray:
    return np.array([p1, p2, p3, p4], dtype=float)


# ---------------------------------------------------------------------------
# Planning (faithful to the MATLAB algorithm)
# ---------------------------------------------------------------------------

def plan_weld(params: GrooveParams | None = None) -> PlanResult:
    p = params or GrooveParams()
    tan_b = p.tan_beta
    g = p.g

    # derived param for layer planning (MATLAB, including the t = h/7 override)
    S = p.aH * math.pi * p.v1 * (p.d ** 2) / (4.0 * p.v2)
    t = math.sqrt(S - g / tan_b) if S > g / tan_b else p.h / p.n_layers_simple
    t = p.h / p.n_layers_simple  # for simplicity, same as MATLAB
    S = (tan_b * t + g) * t
    K = int(math.floor(p.h / t))
    l = S / t  # length of the weld bead in parallelograms

    LN = int(math.ceil(
        (layer_length(K, t, tan_b, g) + layer_length(K - 1, t, tan_b, g))
        / (layer_length(1, t, tan_b, g) + g)
    ))

    # projection vectors used for weld-point placement (MATLAB section 5)
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
    pts_2d: list[list[float]] = []  # [x, z, dx, dz]
    pts_3d: list[list[float]] = []  # [x, y, z, dx, dy, dz]

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
            quad = _quad(
                (xl, z_top), (xr, z_top), (xr_b, z_bot), (xl_b, z_bot),
            )
            emit(i, 1, "center", quad, (0.0, 0.0), up)
            continue

        lp = N // 2
        # left parallelograms j = 1 .. lp  (MATLAB)
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

        # right parallelograms, MATLAB pose loop uses k starting at N-1
        right_welds: list[np.ndarray] = []
        right_quads: list[np.ndarray] = []
        k = N - 1
        for j in range(lp + 1, N):
            x = xr - (N - k) * l
            weld = np.array([x + vr[0], z_top + vr[1]])
            # divider at this x is the inner side of a right bead measured
            # from the right wall: (N-k) steps of l
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
            # MATLAB special case: one left bead, then a center-ish point
            emit(i, 1, "left", left_quads[0], left_welds[0], dl)
            weld2 = np.array([l / 2.0, left_welds[0][1]])
            # remaining trapezoid to the right of the left parallelogram
            quad_c = _quad(
                (xl + l, z_top),
                (xr, z_top),
                (xr_b, z_bot),
                (xl_b + l, z_bot),
            )
            emit(i, 2, "center", quad_c, weld2, up)
            continue

        # N > 2: left beads, right beads, then center trapezoid
        for j, (weld, quad) in enumerate(zip(left_welds, left_quads), start=1):
            emit(i, j, "left", quad, weld, dl)

        for j, (weld, quad) in enumerate(zip(right_welds, right_quads), start=lp + 1):
            emit(i, j, "right", quad, weld, dv)

        # center trapezoid between the innermost left and innermost right
        inner_left_top = xl + lp * l
        inner_left_bot = xl_b + lp * l
        if right_quads:
            inner_right_top = xr - (N - 1 - lp) * l
            inner_right_bot = xr_b - (N - 1 - lp) * l
        else:
            inner_right_top = xr
            inner_right_bot = xr_b
        quad_c = _quad(
            (inner_left_top, z_top),
            (inner_right_top, z_top),
            (inner_right_bot, z_bot),
            (inner_left_bot, z_bot),
        )
        center_weld = 0.5 * (left_welds[-1] + right_welds[-1])
        emit(i, N, "center", quad_c, center_weld, up)

    welding_points = np.array(pts_2d, dtype=float).T if pts_2d else np.zeros((4, 0))
    welding_points_3d = np.array(pts_3d, dtype=float) if pts_3d else np.zeros((0, 6))
    return PlanResult(
        params=p,
        t=t,
        S=S,
        l=l,
        K=K,
        LN=LN,
        beads=beads,
        welding_points=welding_points,
        welding_points_3d=welding_points_3d,
    )


# ---------------------------------------------------------------------------
# 3D drawing
# ---------------------------------------------------------------------------

BEAD_FACE = {
    "left": "#f4a261",
    "right": "#2a9d8f",
    "center": "#e76f51",
}
PLATE_COLOR = "#9aa7b2"
LAYER_COLOR = "#4c78a8"
EDGE_COLOR = "#222222"


def _extrude_quad_xz(quad_xz: np.ndarray, y0: float, y1: float) -> list[np.ndarray]:
    """Extrude a 4-vertex XZ polygon along Y into 6 faces."""
    q = np.asarray(quad_xz, dtype=float)
    front = np.column_stack([q[:, 0], np.full(4, y0), q[:, 1]])
    back = np.column_stack([q[:, 0], np.full(4, y1), q[:, 1]])
    faces = [front, back]
    for i in range(4):
        j = (i + 1) % 4
        faces.append(np.array([front[i], front[j], back[j], back[i]]))
    return faces


def _add_faces(ax, faces, facecolor, edgecolor=EDGE_COLOR, alpha=0.35, lw=0.45) -> None:
    coll = Poly3DCollection(
        faces,
        facecolors=facecolor,
        edgecolors=edgecolor,
        linewidths=lw,
        alpha=alpha,
    )
    ax.add_collection3d(coll)


def groove_corners(p: GrooveParams) -> dict[str, np.ndarray]:
    """MATLAB points A,B,C,D lifted to 3D endpoints y=0 and y=L."""
    tan_b = p.tan_beta
    A = np.array([-p.g / 2.0, 0.0, 0.0])
    B = np.array([p.g / 2.0, 0.0, 0.0])
    C = np.array([p.h * tan_b + p.g / 2.0, 0.0, p.h])
    D = np.array([-p.h * tan_b - p.g / 2.0, 0.0, p.h])
    offset = np.array([0.0, p.weld_length, 0.0])
    return {
        "A": A, "B": B, "C": C, "D": D,
        "A2": A + offset, "B2": B + offset, "C2": C + offset, "D2": D + offset,
    }


def draw_plates(ax, p: GrooveParams, alpha: float = 0.28) -> None:
    """Two workpieces with a V-groove between them, extruded along Y."""
    tan_b = p.tan_beta
    y0, y1 = 0.0, p.weld_length
    x_in_top = p.h * tan_b + p.g / 2.0
    x_out = x_in_top + p.plate_extra

    def plate(sign: float) -> list[np.ndarray]:
        # sign = -1 left, +1 right. Inner face follows the bevel.
        verts = np.array([
            [sign * x_out, y0, 0.0],
            [sign * p.g / 2.0, y0, 0.0],
            [sign * x_in_top, y0, p.h],
            [sign * x_out, y0, p.h],
            [sign * x_out, y1, 0.0],
            [sign * p.g / 2.0, y1, 0.0],
            [sign * x_in_top, y1, p.h],
            [sign * x_out, y1, p.h],
        ])
        faces_idx = [
            (0, 1, 2, 3), (4, 7, 6, 5),
            (0, 3, 7, 4), (1, 5, 6, 2),
            (3, 2, 6, 7), (0, 4, 5, 1),
        ]
        return [verts[list(idx)] for idx in faces_idx]

    _add_faces(ax, plate(-1.0), PLATE_COLOR, alpha=alpha, lw=0.5)
    _add_faces(ax, plate(+1.0), PLATE_COLOR, alpha=alpha, lw=0.5)

    c = groove_corners(p)
    # groove outline
    lines = [
        [c["A"], c["B"]], [c["A"], c["D"]], [c["B"], c["C"]],
        [c["A2"], c["B2"]], [c["A2"], c["D2"]], [c["B2"], c["C2"]],
        [c["A"], c["A2"]], [c["B"], c["B2"]], [c["C"], c["C2"]], [c["D"], c["D2"]],
        [c["D"], c["C"]], [c["D2"], c["C2"]],
    ]
    ax.add_collection3d(Line3DCollection(lines, colors="k", linewidths=1.1))


def draw_layers(ax, plan: PlanResult, alpha: float = 0.18) -> None:
    p = plan.params
    tan_b = p.tan_beta
    y0, y1 = 0.0, p.weld_length
    for i in range(1, plan.K + 1):
        z = i * plan.t
        x = half_width(z, tan_b, p.g)
        quad = np.array([
            [-x, y0, z], [x, y0, z], [x, y1, z], [-x, y1, z],
        ])
        _add_faces(ax, [quad], LAYER_COLOR, edgecolor="#2c3e50", alpha=alpha, lw=0.6)


def draw_segment_dividers(ax, plan: PlanResult) -> None:
    """MATLAB section 3: dividing lines of left/right parallelograms, extruded."""
    p = plan.params
    tan_b = p.tan_beta
    t, l, g = plan.t, plan.l, p.g
    vl = np.array([tan_b * t, -t])
    vr = np.array([-tan_b * t, -t])
    y0, y1 = 0.0, p.weld_length
    lines = []
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
            a = np.array([x, y0, z])
            b = np.array([x + vl[0], y0, z + vl[1]])
            a2 = np.array([x, y1, z])
            b2 = np.array([x + vl[0], y1, z + vl[1]])
            lines.extend([[a, b], [a2, b2], [a, a2], [b, b2]])
        for j in range(lp + 1, N):
            x = xr - (N - j) * l
            a = np.array([x, y0, z])
            b = np.array([x + vr[0], y0, z + vr[1]])
            a2 = np.array([x, y1, z])
            b2 = np.array([x + vr[0], y1, z + vr[1]])
            lines.extend([[a, b], [a2, b2], [a, a2], [b, b2]])
    if lines:
        ax.add_collection3d(Line3DCollection(lines, colors="#333333", linewidths=0.9))


def draw_bead_volumes(ax, plan: PlanResult, alpha: float = 0.32) -> None:
    p = plan.params
    for bead in plan.beads:
        faces = _extrude_quad_xz(bead.quad_xz, 0.0, p.weld_length)
        _add_faces(ax, faces, BEAD_FACE[bead.kind], alpha=alpha, lw=0.35)


def draw_weld_paths(ax, plan: PlanResult) -> None:
    """Straight weld paths along Y through each planned point."""
    L = plan.params.weld_length
    segs = []
    colors = []
    for bead in plan.beads:
        x, z = bead.weld_xz
        segs.append([(x, 0.0, z), (x, L, z)])
        colors.append(BEAD_FACE[bead.kind])
    if segs:
        ax.add_collection3d(Line3DCollection(segs, colors=colors, linewidths=1.6))


def draw_sequence_labels(ax, plan: PlanResult, fontsize: int = 8) -> None:
    """Put sequence numbers on the near Y face so they stay readable in 3D."""
    y = plan.params.weld_length * 0.92
    dz = 0.18 * plan.t
    xs, ys, zs = [], [], []
    for bead in plan.beads:
        x, z = bead.weld_xz
        xs.append(x)
        ys.append(y)
        zs.append(z + dz)
        ax.text(
            x, y, z + dz, str(bead.sequence),
            fontsize=fontsize, color="#111111", weight="bold",
            ha="center", va="center",
        )
    if xs:
        ax.scatter(xs, ys, zs, c="white", s=70, edgecolors="k", linewidths=0.5, depthshade=False)


def _display_direction(direction_xz: np.ndarray, length: float) -> np.ndarray:
    """Scale a 2D XZ torch vector to a visible 3D arrow; keep MATLAB direction."""
    vec = np.array([direction_xz[0], 0.0, direction_xz[1]], dtype=float)
    norm = np.linalg.norm(vec)
    if norm < 1e-12:
        return np.array([0.0, 0.0, length])
    return vec / norm * length


def draw_poses(ax, plan: PlanResult) -> None:
    y = plan.params.weld_length / 2.0
    display_len = max(plan.t * 1.35, 3.0)
    xs, ys, zs, dxs, dys, dzs = [], [], [], [], [], []
    for bead in plan.beads:
        x, z = bead.weld_xz
        d3 = _display_direction(bead.direction_xz, display_len)
        xs.append(x)
        ys.append(y)
        zs.append(z)
        dxs.append(d3[0])
        dys.append(d3[1])
        dzs.append(d3[2])
        ax.scatter(x, y, z, c="b", s=28, depthshade=False, zorder=5)
    if xs:
        ax.quiver(
            xs, ys, zs, dxs, dys, dzs,
            color="r", linewidth=1.6, arrow_length_ratio=0.28, normalize=False,
        )


def set_axes_equal(ax) -> None:
    xlim = np.array(ax.get_xlim3d())
    ylim = np.array(ax.get_ylim3d())
    zlim = np.array(ax.get_zlim3d())
    ranges = np.array([xlim[1] - xlim[0], ylim[1] - ylim[0], zlim[1] - zlim[0]])
    centers = np.array([xlim.mean(), ylim.mean(), zlim.mean()])
    radius = 0.5 * float(ranges.max())
    ax.set_xlim3d(centers[0] - radius, centers[0] + radius)
    ax.set_ylim3d(centers[1] - radius, centers[1] + radius)
    ax.set_zlim3d(max(0.0, centers[2] - radius), centers[2] + radius)
    try:
        ax.set_box_aspect((1, 1, 1))
    except Exception:
        pass


def style_ax(ax, title: str, plan: PlanResult) -> None:
    p = plan.params
    ax.set_title(title, fontsize=12, pad=8)
    ax.set_xlabel("X width (mm)")
    ax.set_ylabel("Y weld length (mm)")
    ax.set_zlabel("Z height (mm)")
    x_span = p.h * p.tan_beta + p.g / 2.0 + p.plate_extra
    ax.set_xlim(-x_span, x_span)
    ax.set_ylim(0.0, p.weld_length)
    ax.set_zlim(0.0, p.h * 1.15)
    ax.view_init(elev=22, azim=-58)
    set_axes_equal(ax)
    ax.tick_params(labelsize=7)


def parameter_text(plan: PlanResult) -> str:
    p = plan.params
    return (
        "V-shape groove parameter\n\n"
        f"Thickness: {p.h:g} mm\n"
        f"Angle: {p.beta:g} degree\n"
        f"Assembly clearance: {p.g:g} mm\n"
        f"Bead thickness: {plan.t:.4g} mm\n"
        f"Number of layer: {plan.K}\n"
        f"Number of weld points: {plan.point_number}\n"
        f"Weld length (Y): {p.weld_length:g} mm"
    )


def build_overview_figure(plan: PlanResult) -> plt.Figure:
    """Five 3D panels matching the original MATLAB layout, plus a parameter box."""
    fig = plt.figure(figsize=(16.5, 10.5), facecolor="white")
    fig.suptitle("V-groove weld planning (3D)", fontsize=16, y=0.98)

    specs = [
        (231, "1. V-shape groove", ["plates"]),
        (232, "2. Plan the layers", ["plates", "layers"]),
        (234, "3. Segmentation", ["plates", "layers", "segments", "beads"]),
        (235, "4. Sequence planning", ["plates", "layers", "segments", "beads", "seq"]),
        (236, "5. Weld points pose", ["plates", "layers", "segments", "beads", "paths", "poses"]),
    ]
    for slot, title, parts in specs:
        ax = fig.add_subplot(slot, projection="3d")
        if "plates" in parts:
            draw_plates(ax, plan.params)
        if "layers" in parts:
            draw_layers(ax, plan)
        if "beads" in parts:
            draw_bead_volumes(ax, plan, alpha=0.22)
        if "segments" in parts:
            draw_segment_dividers(ax, plan)
        if "paths" in parts:
            draw_weld_paths(ax, plan)
        if "seq" in parts:
            draw_sequence_labels(ax, plan, fontsize=6)
        if "poses" in parts:
            draw_poses(ax, plan)
        style_ax(ax, title, plan)

    fig.text(
        0.70, 0.73, parameter_text(plan),
        fontsize=11, va="center", ha="left", family="DejaVu Sans",
        bbox=dict(boxstyle="round", facecolor="white", edgecolor="#888", alpha=0.95),
    )
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    return fig


def _draw_full_scene(ax, plan: PlanResult, with_sequence: bool = True) -> None:
    draw_plates(ax, plan.params, alpha=0.16)
    draw_layers(ax, plan, alpha=0.08)
    draw_bead_volumes(ax, plan, alpha=0.40)
    draw_segment_dividers(ax, plan)
    draw_weld_paths(ax, plan)
    if with_sequence:
        draw_sequence_labels(ax, plan, fontsize=8)
    draw_poses(ax, plan)


def build_detail_figure(plan: PlanResult) -> plt.Figure:
    """Large 3D scene: perspective view + end view (closest to the original 2D)."""
    fig = plt.figure(figsize=(14.5, 7.2), facecolor="white")
    fig.suptitle("V-groove weld points pose (3D)", fontsize=15)

    ax1 = fig.add_subplot(121, projection="3d")
    _draw_full_scene(ax1, plan)
    style_ax(ax1, "Perspective view", plan)

    ax2 = fig.add_subplot(122, projection="3d")
    _draw_full_scene(ax2, plan, with_sequence=True)
    style_ax(ax2, "End view (along weld length Y)", plan)
    ax2.view_init(elev=0, azim=-90)

    fig.text(
        0.01, 0.02, parameter_text(plan),
        fontsize=9, va="bottom",
        bbox=dict(boxstyle="round", facecolor="white", edgecolor="#888", alpha=0.92),
    )
    fig.tight_layout(rect=(0, 0.08, 1, 0.96))
    return fig


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--h", dest="h", type=float, default=16.0, help="groove height, mm")
    parser.add_argument("--beta", type=float, default=30.0, help="bevel angle, deg")
    parser.add_argument("--g", dest="g", type=float, default=1.0, help="root opening, mm")
    parser.add_argument("--weld-length", type=float, default=40.0, help="extrusion length along Y, mm")
    parser.add_argument("--outdir", type=Path, default=Path("figures"), help="directory for PNG output")
    parser.add_argument("--show", action="store_true", help="open interactive windows")
    parser.add_argument("--no-save", action="store_true", help="do not write PNG files")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    params = GrooveParams(h=args.h, beta=args.beta, g=args.g, weld_length=args.weld_length)
    plan = plan_weld(params)

    print("V-shape groove weld planning")
    print(f"  layers K = {plan.K}, bead thickness t = {plan.t:.4f} mm")
    print(f"  parallelogram width l = {plan.l:.4f} mm, max beads LN = {plan.LN}")
    print(f"  weld points = {plan.point_number}")
    print("  welding_points (4 x N) [x; z; dx; dz]:")
    np.set_printoptions(precision=4, suppress=True)
    print(plan.welding_points)
    print("  welding_points_3d (N x 6) [x, y, z, dx, dy, dz]:")
    print(plan.welding_points_3d)

    overview = build_overview_figure(plan)
    detail = build_detail_figure(plan)

    if not args.no_save:
        args.outdir.mkdir(parents=True, exist_ok=True)
        overview_path = args.outdir / "vgroove_weld_3d_overview.png"
        detail_path = args.outdir / "vgroove_weld_3d_detail.png"
        overview.savefig(overview_path, dpi=140)
        detail.savefig(detail_path, dpi=150)
        print(f"  saved {overview_path}")
        print(f"  saved {detail_path}")

    if args.show:
        plt.show()
    else:
        plt.close("all")


if __name__ == "__main__":
    main()
