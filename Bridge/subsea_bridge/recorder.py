"""Record the SubseaSim stream to disk and export the accumulated world-frame map.

The capture format (``.ssim``) is simply the raw wire stream: complete frames written back
to back, so it can be replayed through the same :class:`~subsea_bridge.protocol.StreamParser`.

    python -m subsea_bridge.recorder --out run.ssim --duration 30 --ply map.ply --pcd map.pcd
    python -m subsea_bridge.recorder --input run.ssim --ply map.ply --voxel 0.05
"""

from __future__ import annotations

import argparse
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import BinaryIO, Iterable

import numpy as np
import numpy.typing as npt

from .mapping import MapBuilder, VoxelMap
from .protocol import (
    DEFAULT_PORT,
    MsgType,
    ProtocolError,
    RawFrame,
    StreamClient,
    decode,
    read_recording,
)

# --------------------------------------------------------------------------------------
# Point-cloud file writers (numpy only)
# --------------------------------------------------------------------------------------


def _check(
    xyz: npt.ArrayLike, rgb: npt.ArrayLike | None, intensity: npt.ArrayLike | None
) -> tuple[npt.NDArray[np.float32], npt.NDArray[np.uint8] | None, npt.NDArray[np.uint8] | None]:
    pts = np.asarray(xyz, dtype=np.float32).reshape(-1, 3)
    n = pts.shape[0]
    rgb_arr = None if rgb is None else np.asarray(rgb, dtype=np.uint8).reshape(-1, 3)
    int_arr = None if intensity is None else np.asarray(intensity, dtype=np.uint8).reshape(-1)
    if rgb_arr is not None and rgb_arr.shape[0] != n:
        raise ValueError("rgb must have one row per point")
    if int_arr is not None and int_arr.shape[0] != n:
        raise ValueError("intensity must have one value per point")
    return pts, rgb_arr, int_arr


def write_ply(
    path: str | Path,
    xyz: npt.ArrayLike,
    rgb: npt.ArrayLike | None = None,
    intensity: npt.ArrayLike | None = None,
) -> int:
    """Write a binary little-endian PLY (x y z float [, red green blue uchar] [, intensity uchar])."""
    pts, rgb_arr, int_arr = _check(xyz, rgb, intensity)
    n = pts.shape[0]
    fields: list[tuple[str, str]] = [("x", "<f4"), ("y", "<f4"), ("z", "<f4")]
    props = ["property float x", "property float y", "property float z"]
    if rgb_arr is not None:
        fields += [("red", "u1"), ("green", "u1"), ("blue", "u1")]
        props += ["property uchar red", "property uchar green", "property uchar blue"]
    if int_arr is not None:
        fields.append(("intensity", "u1"))
        props.append("property uchar intensity")
    rec = np.empty(n, dtype=np.dtype(fields))
    rec["x"], rec["y"], rec["z"] = pts[:, 0], pts[:, 1], pts[:, 2]
    if rgb_arr is not None:
        rec["red"], rec["green"], rec["blue"] = rgb_arr[:, 0], rgb_arr[:, 1], rgb_arr[:, 2]
    if int_arr is not None:
        rec["intensity"] = int_arr
    header = "\n".join(
        [
            "ply",
            "format binary_little_endian 1.0",
            "comment SubseaSim map, frame map (REP-103: X forward, Y left, Z up, metres)",
            f"element vertex {n}",
            *props,
            "end_header",
            "",
        ]
    )
    with open(path, "wb") as fh:
        fh.write(header.encode("ascii"))
        fh.write(rec.tobytes())
    return n


def write_pcd(
    path: str | Path,
    xyz: npt.ArrayLike,
    rgb: npt.ArrayLike | None = None,
    intensity: npt.ArrayLike | None = None,
) -> int:
    """Write a binary PCD v0.7 (x y z float32 [, rgb packed uint32] [, intensity float32])."""
    pts, rgb_arr, int_arr = _check(xyz, rgb, intensity)
    n = pts.shape[0]
    fields: list[tuple[str, str]] = [("x", "<f4"), ("y", "<f4"), ("z", "<f4")]
    names, sizes, types = ["x", "y", "z"], ["4", "4", "4"], ["F", "F", "F"]
    if rgb_arr is not None:
        fields.append(("rgb", "<u4"))
        names.append("rgb")
        sizes.append("4")
        types.append("U")
    if int_arr is not None:
        fields.append(("intensity", "<f4"))
        names.append("intensity")
        sizes.append("4")
        types.append("F")
    rec = np.empty(n, dtype=np.dtype(fields))
    rec["x"], rec["y"], rec["z"] = pts[:, 0], pts[:, 1], pts[:, 2]
    if rgb_arr is not None:
        c = rgb_arr.astype(np.uint32)
        rec["rgb"] = (c[:, 0] << 16) | (c[:, 1] << 8) | c[:, 2]
    if int_arr is not None:
        rec["intensity"] = int_arr.astype(np.float32)
    header = "\n".join(
        [
            "# .PCD v0.7 - Point Cloud Data file format",
            "VERSION 0.7",
            "FIELDS " + " ".join(names),
            "SIZE " + " ".join(sizes),
            "TYPE " + " ".join(types),
            "COUNT " + " ".join("1" for _ in names),
            f"WIDTH {n}",
            "HEIGHT 1",
            "VIEWPOINT 0 0 0 1 0 0 0",
            f"POINTS {n}",
            "DATA binary",
            "",
        ]
    )
    with open(path, "wb") as fh:
        fh.write(header.encode("ascii"))
        fh.write(rec.tobytes())
    return n


def export_map(vmap: VoxelMap, ply: str | Path | None = None, pcd: str | Path | None = None) -> None:
    if ply is not None:
        write_ply(ply, vmap.xyz, vmap.rgb, vmap.intensity)
    if pcd is not None:
        write_pcd(pcd, vmap.xyz, vmap.rgb, vmap.intensity)


# --------------------------------------------------------------------------------------
# Recording
# --------------------------------------------------------------------------------------


@dataclass
class RecordStats:
    frames: int = 0
    bytes: int = 0
    by_type: dict[int, int] = field(default_factory=dict)
    decode_errors: int = 0
    first_stamp: float | None = None
    last_stamp: float | None = None
    seconds: float = 0.0

    def summary(self) -> str:
        names = {int(t): t.name.lower() for t in MsgType}
        per = ", ".join(f"{names.get(k, f'type{k}')}={v}" for k, v in sorted(self.by_type.items()))
        span = 0.0 if self.first_stamp is None or self.last_stamp is None else self.last_stamp - self.first_stamp
        return (
            f"{self.frames} frames ({per}), {self.bytes / 1e6:.1f} MB, "
            f"{span:.1f} s of sim time in {self.seconds:.1f} s"
        )


def consume(
    frames: Iterable[RawFrame],
    stats: RecordStats,
    out: BinaryIO | None = None,
    builder: MapBuilder | None = None,
) -> None:
    """Write frames to ``out`` and/or integrate them into ``builder``, updating ``stats``."""
    for frame in frames:
        h = frame.header
        stats.frames += 1
        stats.bytes += 40 + len(frame.payload)
        stats.by_type[h.msg_type] = stats.by_type.get(h.msg_type, 0) + 1
        if stats.first_stamp is None:
            stats.first_stamp = h.stamp
        stats.last_stamp = h.stamp
        if out is not None:
            out.write(frame.to_bytes())
        if builder is not None:
            try:
                builder.process(decode(frame))
            except ProtocolError:
                stats.decode_errors += 1


def record(
    client: StreamClient,
    out_path: str | Path | None = None,
    *,
    duration: float = 0.0,
    max_clouds: int = 0,
    builder: MapBuilder | None = None,
    idle_timeout: float = 0.0,
) -> RecordStats:
    """Record from a live stream until ``duration`` seconds or ``max_clouds`` clouds (0 = no limit).

    ``idle_timeout`` > 0 aborts if nothing at all is received for that long.
    """
    stats = RecordStats()
    t0 = time.monotonic()
    last_rx = t0
    fh: BinaryIO | None = open(out_path, "wb") if out_path is not None else None
    try:
        while True:
            now = time.monotonic()
            if duration > 0 and now - t0 >= duration:
                break
            if max_clouds > 0 and stats.by_type.get(int(MsgType.POINT_CLOUD), 0) >= max_clouds:
                break
            if idle_timeout > 0 and now - last_rx >= idle_timeout:
                break
            frames = client.poll_raw(0.25)
            if frames:
                last_rx = time.monotonic()
                consume(frames, stats, fh, builder)
    except KeyboardInterrupt:
        pass
    finally:
        if fh is not None:
            fh.close()
    stats.seconds = time.monotonic() - t0
    return stats


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Record the SubseaSim stream and/or export the world-frame map")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--input", type=Path, help="read an existing .ssim capture instead of connecting")
    ap.add_argument("--out", type=Path, help="write the raw stream to this .ssim file")
    ap.add_argument("--duration", type=float, default=0.0, help="seconds to record (0 = until Ctrl+C)")
    ap.add_argument("--clouds", type=int, default=0, help="stop after this many point clouds (0 = no limit)")
    ap.add_argument("--ply", type=Path, help="export the accumulated map as binary PLY")
    ap.add_argument("--pcd", type=Path, help="export the accumulated map as binary PCD")
    ap.add_argument("--voxel", type=float, default=0.05, help="map voxel size, metres")
    ap.add_argument("--max-points", type=int, default=5_000_000, help="cap on map points (oldest evicted)")
    ap.add_argument("--map-every", type=int, default=1, help="integrate every Nth cloud into the map")
    args = ap.parse_args(argv)

    want_map = args.ply is not None or args.pcd is not None
    if args.input is None and args.out is None and not want_map:
        ap.error("nothing to do: give --out and/or --ply/--pcd")
    builder = MapBuilder(args.voxel, args.max_points, every=args.map_every) if want_map else None

    if args.input is not None:
        stats = RecordStats()
        t0 = time.monotonic()
        consume(read_recording(args.input), stats, None, builder)
        stats.seconds = time.monotonic() - t0
        print(f"read {args.input}: {stats.summary()}")
    else:
        print(f"recording from {args.host}:{args.port} (Ctrl+C to stop)", flush=True)
        with StreamClient(args.host, args.port) as client:
            stats = record(client, args.out, duration=args.duration, max_clouds=args.clouds, builder=builder)
        print(f"recorded: {stats.summary()}" + (f" -> {args.out}" if args.out else ""))

    if builder is not None:
        export_map(builder.map, args.ply, args.pcd)
        targets = ", ".join(str(p) for p in (args.ply, args.pcd) if p is not None)
        print(
            f"map: {len(builder.map)} points at {args.voxel:g} m voxels from "
            f"{builder.clouds_integrated} clouds -> {targets}"
        )
        if builder.clouds_skipped_no_tf:
            print(f"  ({builder.clouds_skipped_no_tf} clouds skipped: no transform chain to 'map' yet)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
