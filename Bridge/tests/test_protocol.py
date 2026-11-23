from __future__ import annotations

import json
import struct

import numpy as np
import pytest

from subsea_bridge import protocol as p

from .conftest import random_points


def test_layout_constants() -> None:
    assert p.HEADER_SIZE == 40
    assert p.POINT_DTYPE.itemsize == 16
    assert [p.POINT_DTYPE.fields[n][1] for n in ("x", "y", "z", "r", "g", "b", "intensity")] == [0, 4, 8, 12, 13, 14, 15]
    assert p.TRANSFORM_PAYLOAD_SIZE == 120
    assert p.DEFAULT_PORT == 9870


def test_header_bytes_match_spec() -> None:
    """Hand-pack a frame exactly as PROTOCOL.md describes and compare byte for byte."""
    payload = b"\x01\x02\x03"
    frame = p.encode_frame(3, 7, 12.5, "depth_cam", payload)
    expected = (
        b"SSIM"
        + struct.pack("<H", 1)
        + struct.pack("<H", 3)
        + struct.pack("<I", 7)
        + struct.pack("<d", 12.5)
        + b"depth_cam".ljust(16, b"\x00")
        + struct.pack("<I", 3)
        + payload
    )
    assert frame == expected
    h = p.decode_header(frame)
    assert (h.msg_type, h.seq, h.stamp, h.frame_id, h.payload_size, h.version) == (3, 7, 12.5, "depth_cam", 3, 1)


def test_point_cloud_round_trip() -> None:
    pts = random_points(1000)
    frame = p.encode_point_cloud(42, 3.25, "depth_cam", pts, p.FLAG_RGB | p.FLAG_INTENSITY)
    assert len(frame) == 40 + 8 + 16 * 1000
    # Payload prefix: num_points, flags
    assert struct.unpack_from("<II", frame, 40) == (1000, 3)
    msg = p.decode_frame(frame)
    assert isinstance(msg, p.PointCloud)
    assert msg.header.seq == 42 and msg.header.stamp == 3.25 and msg.header.frame_id == "depth_cam"
    assert msg.num_points == 1000 and msg.has_rgb and msg.has_intensity
    assert np.array_equal(msg.points, pts)
    assert np.array_equal(msg.xyz, np.stack([pts["x"], pts["y"], pts["z"]], axis=1))
    assert np.array_equal(msg.rgb, np.stack([pts["r"], pts["g"], pts["b"]], axis=1))
    assert np.array_equal(msg.intensity, pts["intensity"])


def test_point_record_bytes() -> None:
    pts = p.make_points([[1.0, -2.0, 3.5]], [[10, 20, 30]], [200])
    assert pts.tobytes() == struct.pack("<fffBBBB", 1.0, -2.0, 3.5, 10, 20, 30, 200)


def test_point_cloud_decode_is_zero_copy() -> None:
    frame = p.encode_point_cloud(0, 0.0, "depth_cam", random_points(256))
    raw = next(p.iter_frames(frame))
    msg = p.decode(raw)
    assert isinstance(msg, p.PointCloud)
    payload_view = np.frombuffer(raw.payload, dtype=np.uint8)
    for arr in (msg.points, msg.xyz, msg.rgb, msg.intensity):
        assert not arr.flags.owndata
        assert np.shares_memory(arr, payload_view)
    assert not msg.points.flags.writeable


def test_empty_point_cloud() -> None:
    msg = p.decode_frame(p.encode_point_cloud(1, 0.0, "depth_cam", random_points(0), 0))
    assert isinstance(msg, p.PointCloud)
    assert msg.num_points == 0 and msg.xyz.shape == (0, 3)
    assert not msg.has_rgb and not msg.has_intensity


def test_transform_round_trip() -> None:
    frame = p.encode_transform(
        5, 9.75, "base_link", "map", (1.0, 2.0, -30.0), (0.0, 0.0, 0.7071, 0.7071), (0.5, 0.1, 0.0), (0.0, 0.0, 0.2)
    )
    assert len(frame) == 40 + 120
    assert frame[40:56] == b"map".ljust(16, b"\x00")
    assert struct.unpack_from("<13d", frame, 56) == (1.0, 2.0, -30.0, 0.0, 0.0, 0.7071, 0.7071, 0.5, 0.1, 0.0, 0.0, 0.0, 0.2)
    msg = p.decode_frame(frame)
    assert isinstance(msg, p.Transform)
    assert msg.parent_frame == "map" and msg.child_frame == "base_link"
    assert msg.position.tolist() == [1.0, 2.0, -30.0]
    assert msg.orientation.tolist() == [0.0, 0.0, 0.7071, 0.7071]
    assert msg.linear_vel.tolist() == [0.5, 0.1, 0.0]
    assert msg.angular_vel.tolist() == [0.0, 0.0, 0.2]


def test_telemetry_and_command_round_trip() -> None:
    data = {"depth_m": 31.2, "thrusters": [0.1, -0.2], "mode": "pipeline_follow", "new_key": {"nested": True}}
    msg = p.decode_frame(p.encode_telemetry(3, 1.0, data))
    assert isinstance(msg, p.Telemetry) and msg.data == data
    cmd = p.decode_frame(p.encode_command(0, 0.0, {"cmd": "set_turbidity", "value": 0.4}))
    assert isinstance(cmd, p.Command) and cmd.data == {"cmd": "set_turbidity", "value": 0.4}
    assert cmd.header.msg_type == p.MsgType.COMMAND
    # Payload is plain UTF-8 JSON
    assert json.loads(p.encode_telemetry(0, 0.0, {"a": 1})[40:].decode("utf-8")) == {"a": 1}


def test_frame_id_limits() -> None:
    frame = p.encode_frame(3, 0, 0.0, "x" * 16, b"{}")
    assert p.decode_header(frame).frame_id == "x" * 16
    with pytest.raises(p.ProtocolError):
        p.encode_frame(3, 0, 0.0, "x" * 17, b"{}")
    with pytest.raises(p.ProtocolError):
        p.encode_frame(3, 0, 0.0, "caméra", b"{}")


def test_unknown_msg_type_passes_through() -> None:
    frame = p.encode_frame(99, 1, 2.0, "future", b"abc")
    msg = p.decode_frame(frame)
    assert isinstance(msg, p.RawFrame) and msg.payload == b"abc" and msg.header.msg_type == 99
    assert msg.to_bytes() == frame


def test_decode_errors() -> None:
    good = p.encode_point_cloud(0, 0.0, "depth_cam", random_points(4))
    with pytest.raises(p.ProtocolError):
        p.decode_header(b"XXXX" + good[4:])
    with pytest.raises(p.ProtocolError):
        p.decode_header(good[:4] + struct.pack("<H", 2) + good[6:])
    with pytest.raises(p.ProtocolError):
        p.decode_frame(good[:-1])
    hdr = p.decode_header(good)
    # num_points disagrees with the payload size
    bad_payload = struct.pack("<II", 5, 3) + good[48:]
    with pytest.raises(p.ProtocolError):
        p.decode(p.RawFrame(hdr, bad_payload))
    with pytest.raises(p.ProtocolError):
        p.decode_frame(p.encode_frame(p.MsgType.TELEMETRY, 0, 0.0, "", b"[1, 2]"))
    with pytest.raises(p.ProtocolError):
        p.decode_frame(p.encode_frame(p.MsgType.TELEMETRY, 0, 0.0, "", b"\xff\xfe not json"))
    with pytest.raises(p.ProtocolError):
        p.decode_frame(p.encode_frame(p.MsgType.TRANSFORM, 0, 0.0, "a", b"\x00" * 119))
