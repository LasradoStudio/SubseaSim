"""Pure (ROS-free) packing of SubseaSim points into a ``sensor_msgs/PointCloud2`` layout.

Kept separate from ``ros2_bridge`` so it can be unit tested without ROS installed.

Layout, 20 bytes per point, little-endian::

    offset 0   x          FLOAT32
    offset 4   y          FLOAT32
    offset 8   z          FLOAT32
    offset 12  rgb        FLOAT32   (PCL convention: the 4 bytes are B, G, R, 0, i.e. the
                                     uint32 0x00RRGGBB reinterpreted as a float)
    offset 16  intensity  FLOAT32   (0..255, the wire uint8 widened)
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .protocol import FLAG_INTENSITY, FLAG_RGB, POINT_DTYPE, PointArray

# sensor_msgs/msg/PointField datatype constants
PF_INT8, PF_UINT8, PF_INT16, PF_UINT16, PF_INT32, PF_UINT32, PF_FLOAT32, PF_FLOAT64 = range(1, 9)

POINT_STEP = 20

#: Memory layout of one PointCloud2 point. ``rgb`` is stored as its uint32 bit pattern.
CLOUD2_DTYPE = np.dtype(
    [("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgb", "<u4"), ("intensity", "<f4")]
)
assert CLOUD2_DTYPE.itemsize == POINT_STEP


@dataclass(frozen=True, slots=True)
class FieldSpec:
    name: str
    offset: int
    datatype: int
    count: int = 1


POINTCLOUD2_FIELDS: tuple[FieldSpec, ...] = (
    FieldSpec("x", 0, PF_FLOAT32),
    FieldSpec("y", 4, PF_FLOAT32),
    FieldSpec("z", 8, PF_FLOAT32),
    FieldSpec("rgb", 12, PF_FLOAT32),
    FieldSpec("intensity", 16, PF_FLOAT32),
)


@dataclass(frozen=True, slots=True)
class PackedCloud:
    """Everything needed to fill a PointCloud2 except the header."""

    width: int
    height: int
    point_step: int
    row_step: int
    is_bigendian: bool
    is_dense: bool
    fields: tuple[FieldSpec, ...]
    data: bytes


def pack_pointcloud2(points: PointArray, flags: int = FLAG_RGB | FLAG_INTENSITY) -> PackedCloud:
    """Repack wire points (:data:`~subsea_bridge.protocol.POINT_DTYPE`) for PointCloud2.

    If the rgb flag is clear the colour is white; if the intensity flag is clear it is 0.
    The cloud is unorganised (height 1).
    """
    if points.dtype != POINT_DTYPE:
        raise ValueError(f"points must have dtype POINT_DTYPE, got {points.dtype}")
    n = int(points.shape[0])
    out = np.empty(n, dtype=CLOUD2_DTYPE)
    out["x"] = points["x"]
    out["y"] = points["y"]
    out["z"] = points["z"]
    if flags & FLAG_RGB:
        out["rgb"] = (
            (points["r"].astype(np.uint32) << 16)
            | (points["g"].astype(np.uint32) << 8)
            | points["b"].astype(np.uint32)
        )
    else:
        out["rgb"] = 0x00FFFFFF
    if flags & FLAG_INTENSITY:
        out["intensity"] = points["intensity"]
    else:
        out["intensity"] = 0.0
    xyz = out.view("<f4").reshape(-1, 5)[:, :3] if n else np.empty((0, 3), dtype=np.float32)
    return PackedCloud(
        width=n,
        height=1,
        point_step=POINT_STEP,
        row_step=POINT_STEP * n,
        is_bigendian=False,
        is_dense=bool(np.isfinite(xyz).all()),
        fields=POINTCLOUD2_FIELDS,
        data=out.tobytes(),
    )


def stamp_to_sec_nanosec(stamp: float) -> tuple[int, int]:
    """Split float seconds into (sec, nanosec) for ``builtin_interfaces/Time``; clamps at 0."""
    if not math.isfinite(stamp) or stamp <= 0.0:
        return 0, 0
    sec = int(math.floor(stamp))
    nanosec = int(round((stamp - sec) * 1e9))
    if nanosec >= 1_000_000_000:
        sec += 1
        nanosec -= 1_000_000_000
    return sec, nanosec
