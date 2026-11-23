"""Render an accumulated point-cloud map (binary PLY) to a PNG, headlessly.

Produces a plan view in the sensor's own colours and a plan view coloured by elevation, so a
survey can be reviewed or attached to a report without opening a 3D viewer.

    Bridge\\.venv\\Scripts\\python.exe Scripts\\render_map.py Saved\\Survey\\map.ply Docs\\media\\map.png
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

_PLY_TYPES = {
    "char": "i1", "int8": "i1", "uchar": "u1", "uint8": "u1",
    "short": "<i2", "int16": "<i2", "ushort": "<u2", "uint16": "<u2",
    "int": "<i4", "int32": "<i4", "uint": "<u4", "uint32": "<u4",
    "float": "<f4", "float32": "<f4", "double": "<f8", "float64": "<f8",
}


def read_ply(path: Path) -> np.ndarray:
    """Reads the vertex element of a binary little-endian PLY as a structured array."""
    with path.open("rb") as handle:
        if handle.readline().strip() != b"ply":
            raise ValueError(f"{path} is not a PLY file")
        count = 0
        fields: list[tuple[str, str]] = []
        in_vertex = False
        while True:
            line = handle.readline()
            if not line:
                raise ValueError("unterminated PLY header")
            parts = line.decode("ascii", "replace").split()
            if not parts:
                continue
            if parts[0] == "format" and parts[1] != "binary_little_endian":
                raise ValueError(f"unsupported PLY format {parts[1]}")
            elif parts[0] == "element":
                in_vertex = parts[1] == "vertex"
                if in_vertex:
                    count = int(parts[2])
            elif parts[0] == "property" and in_vertex:
                fields.append((parts[2], _PLY_TYPES[parts[1]]))
            elif parts[0] == "end_header":
                break
        return np.fromfile(handle, dtype=np.dtype(fields), count=count)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("ply", type=Path)
    parser.add_argument("png", type=Path)
    parser.add_argument("--title", default="SubseaSim: accumulated depth-camera map")
    args = parser.parse_args()

    cloud = read_ply(args.ply)
    if len(cloud) == 0:
        print("PLY has no points")
        return 1
    names = cloud.dtype.names or ()
    x, y, z = cloud["x"], cloud["y"], cloud["z"]
    if {"red", "green", "blue"} <= set(names):
        colours = np.stack([cloud["red"], cloud["green"], cloud["blue"]], axis=1).astype(np.float32) / 255.0
    else:
        colours = np.full((len(cloud), 3), 0.7, dtype=np.float32)

    extent_x = float(x.max() - x.min())
    extent_y = float(y.max() - y.min())
    print(f"{len(cloud):,} points; extent {extent_x:.1f} x {extent_y:.1f} m; z {z.min():.2f} .. {z.max():.2f} m")

    plt.style.use("dark_background")
    figure, axes = plt.subplots(2, 1, figsize=(16, 9), dpi=120, sharex=True, constrained_layout=True)
    figure.suptitle(f"{args.title}  ({len(cloud):,} points, {extent_x:.0f} m surveyed)", fontsize=14)

    size = max(0.05, min(2.0, 4.0e5 / len(cloud)))
    axes[0].scatter(x, y, c=colours, s=size, linewidths=0, rasterized=True)
    axes[0].set_title("Plan view, sensor colour", loc="left", fontsize=11)

    # Clip the colour range to the central 98 % so a few outliers do not flatten the relief.
    low, high = np.percentile(z, [1.0, 99.0])
    relief = axes[1].scatter(x, y, c=z, cmap="viridis", vmin=low, vmax=high, s=size, linewidths=0, rasterized=True)
    axes[1].set_title("Plan view, elevation", loc="left", fontsize=11)
    figure.colorbar(relief, ax=axes, label="map z (m)", pad=0.01, shrink=0.48, anchor=(0.0, 0.0))

    for axis in axes:
        axis.set_aspect("equal")
        axis.set_ylabel("y (m)")
        axis.set_facecolor("#05080c")
    axes[1].set_xlabel("x (m)")

    args.png.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.png)
    print(f"wrote {args.png}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
