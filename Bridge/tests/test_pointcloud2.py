from __future__ import annotations

import struct
import sys

import numpy as np
import pytest

from subsea_bridge import protocol as p
from subsea_bridge.pointcloud2 import (
    PF_FLOAT32,
    POINT_STEP,
    pack_pointcloud2,
    stamp_to_sec_nanosec,
)

from .conftest import random_points


def test_layout_matches_pointfield_contract() -> None:
    packed = pack_pointcloud2(random_points(10))
    assert [(f.name, f.offset, f.datatype, f.count) for f in packed.fields] == [
        ("x", 0, PF_FLOAT32, 1),
        ("y", 4, PF_FLOAT32, 1),
        ("z", 8, PF_FLOAT32, 1),
        ("rgb", 12, PF_FLOAT32, 1),
        ("intensity", 16, PF_FLOAT32, 1),
    ]
    assert PF_FLOAT32 == 7  # sensor_msgs/msg/PointField.FLOAT32
    assert packed.point_step == POINT_STEP == 20
    assert packed.width == 10 and packed.height == 1
    assert packed.row_step == 200 == len(packed.data)
    assert packed.is_bigendian is False and packed.is_dense is True


def test_single_point_bytes() -> None:
    pts = p.make_points([[1.5, -2.25, 3.0]], [[0x11, 0x22, 0x33]], [200])
    packed = pack_pointcloud2(pts)
    x, y, z = struct.unpack_from("<fff", packed.data, 0)
    assert (x, y, z) == (1.5, -2.25, 3.0)
    # PCL packed rgb: uint32 0x00RRGGBB, i.e. bytes B, G, R, 0 in memory
    assert packed.data[12:16] == bytes([0x33, 0x22, 0x11, 0x00])
    assert struct.unpack_from("<I", packed.data, 12)[0] == 0x112233
    assert struct.unpack_from("<f", packed.data, 16)[0] == 200.0


def test_bulk_values_round_trip() -> None:
    pts = random_points(5000, seed=3)
    packed = pack_pointcloud2(pts, p.FLAG_RGB | p.FLAG_INTENSITY)
    rec = np.frombuffer(
        packed.data, dtype=np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgb", "<u4"), ("i", "<f4")])
    )
    assert np.array_equal(rec["x"], pts["x"]) and np.array_equal(rec["y"], pts["y"]) and np.array_equal(rec["z"], pts["z"])
    assert np.array_equal((rec["rgb"] >> 16) & 0xFF, pts["r"])
    assert np.array_equal((rec["rgb"] >> 8) & 0xFF, pts["g"])
    assert np.array_equal(rec["rgb"] & 0xFF, pts["b"])
    assert np.array_equal(rec["rgb"] >> 24, np.zeros(5000, dtype=np.uint32))
    assert np.array_equal(rec["i"], pts["intensity"].astype(np.float32))


def test_flags_control_colour_and_intensity() -> None:
    pts = random_points(16)
    rec_dtype = np.dtype([("xyz", "<f4", 3), ("rgb", "<u4"), ("i", "<f4")])
    no_attrs = np.frombuffer(pack_pointcloud2(pts, 0).data, dtype=rec_dtype)
    assert np.all(no_attrs["rgb"] == 0x00FFFFFF) and np.all(no_attrs["i"] == 0.0)
    rgb_only = np.frombuffer(pack_pointcloud2(pts, p.FLAG_RGB).data, dtype=rec_dtype)
    assert np.all(rgb_only["i"] == 0.0) and not np.all(rgb_only["rgb"] == 0x00FFFFFF)


def test_empty_and_non_finite() -> None:
    empty = pack_pointcloud2(random_points(0))
    assert empty.width == 0 and empty.row_step == 0 and empty.data == b"" and empty.is_dense
    pts = p.make_points([[np.nan, 0, 0], [1, 2, 3]])
    assert pack_pointcloud2(pts).is_dense is False
    with pytest.raises(ValueError):
        pack_pointcloud2(np.zeros(3, dtype=np.float32))  # type: ignore[arg-type]


def test_accepts_decoded_read_only_points() -> None:
    msg = p.decode_frame(p.encode_point_cloud(0, 0.0, "depth_cam", random_points(32)))
    assert isinstance(msg, p.PointCloud)
    assert len(pack_pointcloud2(msg.points, msg.flags).data) == 32 * 20


def test_stamp_split() -> None:
    assert stamp_to_sec_nanosec(0.0) == (0, 0)
    assert stamp_to_sec_nanosec(12.5) == (12, 500_000_000)
    assert stamp_to_sec_nanosec(-3.0) == (0, 0)
    assert stamp_to_sec_nanosec(float("nan")) == (0, 0)
    sec, nsec = stamp_to_sec_nanosec(1_795_000_000.123456)
    assert sec == 1_795_000_000 and abs(nsec - 123_456_000) < 1000
    sec, nsec = stamp_to_sec_nanosec(4.9999999999)
    assert (sec, nsec) == (5, 0)


def test_package_imports_without_ros() -> None:
    """Only ros2_bridge may import ROS; everything else must work on a plain interpreter."""
    import subsea_bridge  # noqa: F401
    import subsea_bridge.benchmark  # noqa: F401
    import subsea_bridge.mock_sim  # noqa: F401
    import subsea_bridge.pointcloud2  # noqa: F401
    import subsea_bridge.recorder  # noqa: F401

    assert "rclpy" not in sys.modules
    assert "subsea_bridge.ros2_bridge" not in sys.modules
