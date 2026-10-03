"""把黄、绿两块连通域收成同一平面上的点，调用 PCL 补上中间缺口。"""

import argparse
import subprocess
import tempfile

import numpy as np
from PIL import Image


def plate_points(rgb: np.ndarray) -> np.ndarray:
    red = rgb[:, :, 0].astype(np.int16)
    green = rgb[:, :, 1].astype(np.int16)
    blue = rgb[:, :, 2].astype(np.int16)
    yellow = (blue < 120) & (red > 110) & (green > 110) & (red + green > blue * 3) & (np.abs(red - green) < 80)
    leaf = (green > red + 25) & (green > blue + 25) & (blue < 130) & (red < 150) & (green > 80) & ~yellow
    rows, cols = np.where(yellow | leaf)
    return np.column_stack([cols, rows, np.zeros(len(cols))]).astype(np.float32)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("image")
    parser.add_argument("filler", help="编译好的 fill_planar_gap")
    parser.add_argument("-o", "--output", required=True)
    args = parser.parse_args()

    image = np.array(Image.open(args.image))
    rgb = image[:, :, :3]
    points = plate_points(rgb)
    with tempfile.TemporaryDirectory() as directory:
        source = f"{directory}/in.xyz"
        filled_path = f"{directory}/out.xyz"
        np.savetxt(source, points, fmt="%.3f")
        subprocess.run([args.filler, source, filled_path], check=True)
        filled = np.loadtxt(filled_path)

    height, width = rgb.shape[:2]
    cols = np.rint(filled[:, 0]).astype(int)
    rows = np.rint(filled[:, 1]).astype(int)
    keep = (cols >= 0) & (cols < width) & (rows >= 0) & (rows < height)
    out = rgb.copy()
    out[rows[keep], cols[keep]] = (220, 30, 30)
    Image.fromarray(out).save(args.output)
    print(f"input {len(points)} filled {len(filled)}")


if __name__ == "__main__":
    main()
