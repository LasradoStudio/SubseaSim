from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from subsea_bridge import protocol as p
from subsea_bridge.mapping import MapBuilder
from subsea_bridge.recorder import RecordStats, consume, main, write_pcd, write_ply

from .conftest import tick_frames


def _split_header(path: Path, terminator: bytes) -> tuple[list[str], bytes]:
    blob = path.read_bytes()
    end = blob.index(terminator) + len(terminator)
    return blob[:end].decode("ascii").splitlines(), blob[end:]


def test_write_ply_binary_round_trip(tmp_path: Path) -> None:
    rng = np.random.default_rng(0)
    xyz = rng.normal(size=(257, 3)).astype(np.float32)
    rgb = rng.integers(0, 256, size=(257, 3), dtype=np.uint8)
    inten = rng.integers(0, 256, size=257, dtype=np.uint8)
    path = tmp_path / "map.ply"
    assert write_ply(path, xyz, rgb, inten) == 257

    header, body = _split_header(path, b"end_header\n")
    assert header[0] == "ply"
    assert header[1] == "format binary_little_endian 1.0"
    assert "element vertex 257" in header
    props = [line for line in header if line.startswith("property")]
    assert props == [
        "property float x",
        "property float y",
        "property float z",
        "property uchar red",
        "property uchar green",
        "property uchar blue",
        "property uchar intensity",
    ]
    assert header[-1] == "end_header"
    dtype = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("r", "u1"), ("g", "u1"), ("b", "u1"), ("i", "u1")])
    assert len(body) == 257 * dtype.itemsize == 257 * 16
    rec = np.frombuffer(body, dtype=dtype)
    assert np.array_equal(np.stack([rec["x"], rec["y"], rec["z"]], axis=1), xyz)
    assert np.array_equal(np.stack([rec["r"], rec["g"], rec["b"]], axis=1), rgb)
    assert np.array_equal(rec["i"], inten)


def test_write_ply_xyz_only_and_empty(tmp_path: Path) -> None:
    path = tmp_path / "xyz.ply"
    write_ply(path, [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]])
    header, body = _split_header(path, b"end_header\n")
    assert [line for line in header if line.startswith("property")] == [
        "property float x",
        "property float y",
        "property float z",
    ]
    assert np.frombuffer(body, dtype="<f4").tolist() == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]

    empty = tmp_path / "empty.ply"
    assert write_ply(empty, np.empty((0, 3))) == 0
    header, body = _split_header(empty, b"end_header\n")
    assert "element vertex 0" in header and body == b""

    with pytest.raises(ValueError):
        write_ply(path, [[0, 0, 0]], rgb=[[1, 2, 3], [4, 5, 6]])


def test_write_pcd_binary_round_trip(tmp_path: Path) -> None:
    xyz = np.array([[1.0, 2.0, 3.0], [-1.0, -2.0, -3.0]], dtype=np.float32)
    rgb = np.array([[255, 128, 1], [0, 16, 32]], dtype=np.uint8)
    path = tmp_path / "map.pcd"
    assert write_pcd(path, xyz, rgb, [7, 250]) == 2
    header, body = _split_header(path, b"DATA binary\n")
    kv = {line.split(" ", 1)[0]: line.split(" ", 1)[1] for line in header if not line.startswith("#")}
    assert kv["FIELDS"] == "x y z rgb intensity"
    assert kv["SIZE"] == "4 4 4 4 4" and kv["TYPE"] == "F F F U F" and kv["COUNT"] == "1 1 1 1 1"
    assert kv["WIDTH"] == "2" and kv["HEIGHT"] == "1" and kv["POINTS"] == "2"
    rec = np.frombuffer(body, dtype=np.dtype([("xyz", "<f4", 3), ("rgb", "<u4"), ("i", "<f4")]))
    assert np.array_equal(rec["xyz"], xyz)
    assert rec["rgb"].tolist() == [0xFF8001, 0x001020]
    assert rec["i"].tolist() == [7.0, 250.0]


def test_consume_writes_replayable_capture(tmp_path: Path) -> None:
    frames = [f for seq in range(6) for f in tick_frames(seq)]
    cap = tmp_path / "run.ssim"
    stats = RecordStats()
    builder = MapBuilder(voxel=0.5)
    with open(cap, "wb") as fh:
        consume(p.iter_frames(b"".join(frames)), stats, fh, builder)
    assert cap.read_bytes() == b"".join(frames)
    assert stats.frames == 24 and stats.by_type == {1: 6, 2: 12, 3: 6}
    assert stats.first_stamp == 0.0 and stats.last_stamp == pytest.approx(0.5)
    assert builder.clouds_integrated == 6 and len(builder.map) > 0
    assert "point_cloud=6" in stats.summary()


def test_cli_offline_export(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    cap = tmp_path / "run.ssim"
    cap.write_bytes(b"".join(f for seq in range(4) for f in tick_frames(seq, n=200)))
    ply, pcd = tmp_path / "m.ply", tmp_path / "m.pcd"
    assert main(["--input", str(cap), "--ply", str(ply), "--pcd", str(pcd), "--voxel", "0.25"]) == 0
    out = capsys.readouterr().out
    assert "4 clouds" in out
    header, body = _split_header(ply, b"end_header\n")
    n = int(next(line for line in header if line.startswith("element vertex")).split()[-1])
    assert 0 < n <= 800 and len(body) == n * 16
    assert pcd.stat().st_size > 0
