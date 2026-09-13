"""Minimal XYZ PLY reader/writer (ASCII and binary little-endian)."""

from __future__ import annotations

from pathlib import Path
from typing import Iterable

import numpy as np


def load_ply(path: str | Path) -> np.ndarray:
    path = Path(path)
    if not path.is_file():
        raise FileNotFoundError(f"PLY not found: {path}")

    with path.open("rb") as handle:
        header_lines: list[str] = []
        while True:
            line = handle.readline()
            if not line:
                raise ValueError(f"Invalid PLY header: {path}")
            text = line.decode("ascii", errors="replace").strip()
            header_lines.append(text)
            if text == "end_header":
                break
        payload = handle.read()

    if not header_lines or header_lines[0] != "ply":
        raise ValueError(f"Not a PLY file: {path}")

    fmt = "ascii"
    vertex_count = 0
    props: list[tuple[str, str]] = []
    in_vertex = False
    for line in header_lines[1:]:
        if line.startswith("format "):
            fmt = line.split()[1]
        elif line.startswith("element vertex"):
            vertex_count = int(line.split()[-1])
            in_vertex = True
        elif line.startswith("element "):
            in_vertex = False
        elif line.startswith("property ") and in_vertex:
            parts = line.split()
            props.append((parts[1], parts[2]))

    names = [name for _, name in props]
    for axis in ("x", "y", "z"):
        if axis not in names:
            raise ValueError(f"PLY is missing property '{axis}': {path}")

    if vertex_count <= 0:
        raise ValueError(f"PLY has no vertices: {path}")

    if fmt == "ascii":
        text = payload.decode("ascii", errors="replace")
        rows = []
        for raw in text.splitlines():
            raw = raw.strip()
            if not raw:
                continue
            rows.append([float(tok) for tok in raw.split()[: len(props)]])
            if len(rows) >= vertex_count:
                break
        data = np.asarray(rows, dtype=np.float64)
    elif fmt in ("binary_little_endian", "binary_big_endian"):
        dtype_map = {
            "char": "i1",
            "uchar": "u1",
            "short": "i2",
            "ushort": "u2",
            "int": "i4",
            "uint": "u4",
            "float": "f4",
            "double": "f8",
            "int8": "i1",
            "uint8": "u1",
            "int16": "i2",
            "uint16": "u2",
            "int32": "i4",
            "uint32": "u4",
            "float32": "f4",
            "float64": "f8",
        }
        endian = "<" if fmt == "binary_little_endian" else ">"
        descr = [(name, endian + dtype_map[typ]) for typ, name in props]
        data = np.frombuffer(payload, dtype=np.dtype(descr), count=vertex_count)
        data = np.column_stack([data["x"], data["y"], data["z"]]).astype(np.float64)
        return np.ascontiguousarray(data)
    else:
        raise ValueError(f"Unsupported PLY format '{fmt}': {path}")

    idx = [names.index(axis) for axis in ("x", "y", "z")]
    return np.ascontiguousarray(data[:, idx], dtype=np.float64)


def save_ply(path: str | Path, points: np.ndarray, colors: np.ndarray | None = None) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    header = [
        "ply",
        "format ascii 1.0",
        f"element vertex {len(pts)}",
        "property float x",
        "property float y",
        "property float z",
    ]
    if colors is not None:
        header.extend(
            [
                "property uchar red",
                "property uchar green",
                "property uchar blue",
            ]
        )
    header.append("end_header")

    with path.open("w", encoding="ascii") as handle:
        handle.write("\n".join(header) + "\n")
        if colors is None:
            for x, y, z in pts:
                handle.write(f"{x:.6f} {y:.6f} {z:.6f}\n")
            return
        rgb = np.asarray(colors)
        if rgb.max() <= 1.0:
            rgb = (np.clip(rgb, 0.0, 1.0) * 255.0).astype(np.uint8)
        else:
            rgb = np.clip(rgb, 0, 255).astype(np.uint8)
        for (x, y, z), (r, g, b) in zip(pts, rgb):
            handle.write(f"{x:.6f} {y:.6f} {z:.6f} {int(r)} {int(g)} {int(b)}\n")


def save_csv(path: str | Path, rows: np.ndarray, header: Iterable[str]) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    array = np.asarray(rows, dtype=np.float64)
    np.savetxt(path, array, delimiter=",", header=",".join(header), comments="")
