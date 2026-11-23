from __future__ import annotations

import os
import struct

import numpy as np
import pytest

from subsea_bridge import protocol as p

from .conftest import tick_frames


def _types(frames: list[p.RawFrame]) -> list[int]:
    return [f.header.msg_type for f in frames]


def _feed(parser: p.StreamParser, data: bytes, chunk: int) -> list[p.RawFrame]:
    out: list[p.RawFrame] = []
    for off in range(0, len(data), chunk):
        out += parser.feed(data[off : off + chunk])
    return out


@pytest.mark.parametrize("chunk", [1, 3, 7, 39, 40, 41, 1000, 1 << 20])
def test_clean_stream_any_chunking(chunk: int) -> None:
    frames = [f for seq in range(5) for f in tick_frames(seq)]
    parser = p.StreamParser()
    out = _feed(parser, b"".join(frames), chunk)
    assert [f.to_bytes() for f in out] == frames
    assert parser.bytes_discarded == 0 and parser.resyncs == 0 and parser.buffered == 0


def test_partial_frame_is_held_until_complete() -> None:
    frame = tick_frames(0)[2]
    parser = p.StreamParser()
    assert parser.feed(frame[:20]) == []  # partial header
    assert parser.feed(frame[20:300]) == []  # header + partial payload
    assert parser.buffered == 300
    out = parser.feed(frame[300:])
    assert len(out) == 1 and out[0].to_bytes() == frame
    assert parser.bytes_discarded == 0


def test_resync_after_leading_and_interleaved_garbage() -> None:
    rng = np.random.default_rng(1)
    ticks = [tick_frames(seq) for seq in range(4)]
    stream = b""
    for frames in ticks:
        for f in frames:
            stream += rng.integers(0, 256, size=int(rng.integers(1, 200)), dtype=np.uint8).tobytes().replace(b"SSIM", b"ssim")
            stream += f
    for chunk in (1, 13, 4096):
        parser = p.StreamParser()
        out = _feed(parser, stream, chunk)
        assert [f.to_bytes() for f in out] == [f for frames in ticks for f in frames]
        assert parser.resyncs == 16
        assert parser.bytes_discarded == len(stream) - sum(len(f) for frames in ticks for f in frames)


def test_resync_skips_false_magic_and_bad_headers() -> None:
    good = tick_frames(0)
    junk = (
        b"SSIM"  # bare magic, then nonsense
        + b"\x07" * 50
        + p.encode_frame(3, 0, 0.0, "x", b"{}")[:4]
        + struct.pack("<H", 2)  # wrong version
        + b"\x00" * 60
        + b"SSIM" + struct.pack("<HHId16sI", 1, 3, 0, 0.0, b"x", p.MAX_PAYLOAD_SIZE + 1)  # absurd size
        + b"SSIM" + struct.pack("<HHId16sI", 1, 2, 0, 0.0, b"x", 119)  # transform with wrong size
        + b"SSIM" + struct.pack("<HHId16sI", 1, 1, 0, 0.0, b"x", 8 + 15)  # cloud size not 8 + 16n
        + b"SSIM" + struct.pack("<HHId16sI", 1, 1, 0, 0.0, b"x", 8 + 32) + struct.pack("<II", 3, 0)  # n mismatch
        + b"SS"
    )
    parser = p.StreamParser()
    out = _feed(parser, junk + b"".join(good) + junk + b"".join(good), 17)
    assert [f.to_bytes() for f in out] == good + good
    assert parser.resyncs == 2


def test_truncated_frame_followed_by_valid_frames() -> None:
    """A frame cut short mid-payload must not swallow the frames that follow it."""
    a = tick_frames(0, n=500)  # the cloud is 8 KB
    b = tick_frames(1, n=20)
    c = tick_frames(2, n=20)
    stream = a[0] + a[1] + a[2][:1000] + b"".join(b) + b"".join(c) + b"".join(tick_frames(3, n=600))
    for chunk in (1, 64, 1 << 16):
        parser = p.StreamParser()
        out = _feed(parser, stream, chunk)
        got = [f.to_bytes() for f in out]
        assert got[:2] == a[:2]
        assert got[2 : 2 + 8] == b + c
        assert len(got) == 2 + 8 + 4
        assert parser.truncated_frames >= 1 and parser.resyncs >= 1
        for f in out:
            p.decode(f)  # everything delivered is well formed


def test_truncated_header_followed_by_valid_frame() -> None:
    good = tick_frames(0)
    parser = p.StreamParser()
    out = _feed(parser, good[0][:25] + b"".join(good), 5)
    assert [f.to_bytes() for f in out] == good
    assert parser.resyncs == 1


def test_magic_split_across_chunks_after_garbage() -> None:
    frame = tick_frames(0)[3]
    parser = p.StreamParser()
    assert parser.feed(b"\xaa" * 10 + frame[:2]) == []
    assert parser.feed(frame[2:3]) == []
    out = parser.feed(frame[3:])
    assert len(out) == 1 and out[0].to_bytes() == frame
    assert parser.bytes_discarded == 10


def test_garbage_only_does_not_grow_buffer() -> None:
    parser = p.StreamParser()
    for _ in range(50):
        assert parser.feed(os.urandom(4096).replace(b"SSIM", b"....")) == []
    assert parser.buffered <= 3
    assert parser.bytes_discarded >= 50 * 4096 - 3


def test_magic_bytes_inside_payload_do_not_break_framing() -> None:
    """'SSIM' appearing inside point data or JSON is not mistaken for a header."""
    pts = p.make_points(np.zeros((8, 3), dtype=np.float32))
    raw = bytearray(pts.tobytes())
    raw[16:20] = b"SSIM"  # x of point 1
    cloud = p.encode_point_cloud(0, 0.0, "depth_cam", np.frombuffer(bytes(raw), dtype=p.POINT_DTYPE))
    tel = p.encode_telemetry(0, 0.0, {"note": "SSIM SSIM SSIM"})
    parser = p.StreamParser()
    out = _feed(parser, cloud + tel + cloud, 9)
    assert [f.to_bytes() for f in out] == [cloud, tel, cloud]
    assert parser.bytes_discarded == 0 and parser.truncated_frames == 0


def test_reset_drops_partial_data() -> None:
    frame = tick_frames(0)[2]
    parser = p.StreamParser()
    parser.feed(frame[:100])
    parser.reset()
    out = parser.feed(frame)
    assert len(out) == 1 and parser.resyncs == 0


def test_iter_frames_and_read_recording(tmp_path) -> None:  # type: ignore[no-untyped-def]
    frames = [f for seq in range(3) for f in tick_frames(seq)]
    blob = b"".join(frames)
    assert _types(list(p.iter_frames(blob, chunk_size=100))) == [2, 2, 1, 3] * 3
    path = tmp_path / "cap.ssim"
    path.write_bytes(blob)
    assert [f.to_bytes() for f in p.read_recording(path, chunk_size=77)] == frames
