"""Decode-throughput benchmark for the SubseaSim stream.

Two measurements:

* **offline** - pre-encoded frames are pushed through the stream parser and decoder in
  socket-sized chunks, with no network and no simulator in the loop. This is the ceiling
  of the Python client.
* **live** - connect to a simulator (``--spawn-mock`` starts the mock in a child process,
  otherwise ``--host/--port``) and measure what actually arrives: rates, inter-arrival
  jitter, per-frame decode time, and end-to-end latency when the telemetry carries the
  optional ``wall_time_s`` key (the mock does; only meaningful on the same machine).

    python -m subsea_bridge.benchmark --spawn-mock
    python -m subsea_bridge.benchmark --mode live --host 127.0.0.1 --port 9870 --seconds 10
"""

from __future__ import annotations

import argparse
import socket
import subprocess
import sys
import time
from dataclasses import dataclass, field

import numpy as np

from .protocol import (
    DEFAULT_PORT,
    HEADER_SIZE,
    PointCloud,
    StreamClient,
    StreamParser,
    Telemetry,
    decode,
    encode_point_cloud,
    encode_telemetry,
    encode_transform,
    make_points,
)


@dataclass
class Result:
    name: str
    seconds: float = 0.0
    frames: int = 0
    clouds: int = 0
    points: int = 0
    bytes: int = 0
    decode_ms: list[float] = field(default_factory=list)
    interarrival_ms: list[float] = field(default_factory=list)
    latency_ms: list[float] = field(default_factory=list)
    resyncs: int = 0

    def rows(self) -> list[tuple[str, str]]:
        s = max(self.seconds, 1e-9)
        rows = [
            ("duration", f"{self.seconds:.2f} s"),
            ("frames (all types)", f"{self.frames}  ({self.frames / s:,.1f} /s)"),
            ("point clouds", f"{self.clouds}  ({self.clouds / s:,.1f} /s)"),
            ("points", f"{self.points:,}  ({self.points / s / 1e6:,.2f} M/s)"),
            ("data", f"{self.bytes / 1e6:,.1f} MB  ({self.bytes / s / 1e6:,.1f} MB/s)"),
        ]
        for label, vals in (
            ("cloud decode", self.decode_ms),
            ("cloud inter-arrival", self.interarrival_ms),
            ("end-to-end latency", self.latency_ms),
        ):
            if vals:
                p50, p90, p99 = np.percentile(np.asarray(vals), [50, 90, 99])
                rows.append((f"{label} p50/p90/p99", f"{p50:.3f} / {p90:.3f} / {p99:.3f} ms  (max {max(vals):.3f})"))
        if self.resyncs:
            rows.append(("resyncs", str(self.resyncs)))
        return rows


def format_table(results: list[Result]) -> str:
    lines: list[str] = []
    for res in results:
        rows = res.rows()
        w = max(len(k) for k, _ in rows)
        lines.append(f"== {res.name} ==")
        lines += [f"  {k:<{w}}  {v}" for k, v in rows]
        lines.append("")
    return "\n".join(lines)


def synth_tick(seq: int, num_points: int, rng: np.random.Generator) -> bytes:
    """One tick's worth of frames (2 transforms, cloud, telemetry) with random points."""
    xyz = rng.uniform(-6.0, 6.0, size=(num_points, 3)).astype(np.float32)
    rgb = rng.integers(0, 256, size=(num_points, 3), dtype=np.uint8)
    inten = rng.integers(0, 256, size=num_points, dtype=np.uint8)
    stamp = seq * 0.05
    return b"".join(
        (
            encode_transform(2 * seq, stamp, "base_link", "map", (seq * 0.02, 0, -30), (0, 0, 0, 1)),
            encode_transform(2 * seq + 1, stamp, "depth_cam", "base_link", (0.4, 0, -0.2), (0, 0.3, 0, 0.954)),
            encode_point_cloud(seq, stamp, "depth_cam", make_points(xyz, rgb, inten)),
            encode_telemetry(seq, stamp, {"depth_m": 30.0, "points": num_points, "thrusters": [0.0] * 8}),
        )
    )


def bench_offline(num_points: int = 76_800, ticks: int = 300, chunk: int = 1 << 16, seed: int = 0) -> Result:
    """Parser + decoder throughput on an in-memory stream fed in ``chunk``-byte pieces."""
    rng = np.random.default_rng(seed)
    unique = [synth_tick(i, num_points, rng) for i in range(8)]
    stream = memoryview(b"".join(unique[i % len(unique)] for i in range(ticks)))
    parser = StreamParser()
    res = Result(f"offline decode ({num_points:,} pts/cloud, {chunk // 1024} KiB chunks)")
    checksum = 0.0
    t0 = time.perf_counter()
    for off in range(0, len(stream), chunk):
        for frame in parser.feed(stream[off : off + chunk]):
            t1 = time.perf_counter()
            msg = decode(frame)
            res.frames += 1
            if isinstance(msg, PointCloud):
                # Touch the data so the zero-copy view is actually exercised.
                checksum += float(msg.xyz[0, 0]) + float(msg.xyz[-1, 2]) if msg.num_points else 0.0
                res.clouds += 1
                res.points += msg.num_points
                res.decode_ms.append((time.perf_counter() - t1) * 1e3)
    res.seconds = time.perf_counter() - t0
    res.bytes = len(stream)
    res.resyncs = parser.resyncs
    return res


def bench_live(host: str, port: int, seconds: float = 8.0, warmup: float = 1.0, name: str | None = None) -> Result:
    """Measure a live stream for ``seconds`` after ``warmup`` seconds of discarded data."""
    res = Result(name or f"live stream {host}:{port}")
    with StreamClient(host, port, reconnect=True) as client:
        deadline = time.monotonic() + 15.0
        while not client.connected and time.monotonic() < deadline:
            client.poll_raw(0.2)
        if not client.connected:
            raise ConnectionError(f"could not connect to {host}:{port}: {client.last_error}")
        t_end = time.monotonic() + warmup
        while time.monotonic() < t_end:
            client.poll_raw(0.1)
        resync0 = client.parser.resyncs
        last_cloud: float | None = None
        t0 = time.perf_counter()
        while (now := time.perf_counter()) - t0 < seconds:
            frames = client.poll_raw(0.1)
            t_rx = time.time()
            for frame in frames:
                t1 = time.perf_counter()
                msg = decode(frame)
                res.frames += 1
                res.bytes += HEADER_SIZE + len(frame.payload)
                if isinstance(msg, PointCloud):
                    res.clouds += 1
                    res.points += msg.num_points
                    res.decode_ms.append((time.perf_counter() - t1) * 1e3)
                    if last_cloud is not None:
                        res.interarrival_ms.append((t1 - last_cloud) * 1e3)
                    last_cloud = t1
                elif isinstance(msg, Telemetry):
                    sent = msg.data.get("wall_time_s")
                    if isinstance(sent, (int, float)):
                        res.latency_ms.append((t_rx - float(sent)) * 1e3)
        res.seconds = now - t0
        res.resyncs = client.parser.resyncs - resync0
    return res


def _free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(("127.0.0.1", 0))
        return int(s.getsockname()[1])


def bench_mock(seconds: float, rate: float, width: int, height: int) -> Result:
    """Spawn the mock sim in a child process (own GIL) and benchmark against it."""
    port = _free_port()
    proc = subprocess.Popen(
        [
            sys.executable,
            "-m",
            "subsea_bridge.mock_sim",
            "--port",
            str(port),
            "--rate",
            str(rate),
            "--width",
            str(width),
            "--height",
            str(height),
            "--duration",
            str(seconds + 40.0),  # safety net: the child exits by itself even if we die
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    try:
        return bench_live("127.0.0.1", port, seconds, name=f"live mock sim ({width}x{height} @ {rate:g} Hz requested)")
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=5.0)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="SubseaSim stream decode benchmark")
    ap.add_argument("--mode", choices=("offline", "live", "both"), default="both")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--spawn-mock", action="store_true", help="benchmark against a mock sim child process")
    ap.add_argument("--seconds", type=float, default=8.0, help="live measurement window")
    ap.add_argument("--rate", type=float, default=10.0, help="mock sim rate (with --spawn-mock)")
    ap.add_argument("--width", type=int, default=320)
    ap.add_argument("--height", type=int, default=240)
    ap.add_argument("--points", type=int, default=76_800, help="points per cloud for the offline test")
    ap.add_argument("--ticks", type=int, default=300, help="ticks for the offline test")
    args = ap.parse_args(argv)

    results: list[Result] = []
    if args.mode in ("offline", "both"):
        results.append(bench_offline(args.points, args.ticks, chunk=1 << 16))
        results.append(bench_offline(args.points, args.ticks, chunk=1 << 20))
    if args.mode in ("live", "both"):
        try:
            if args.spawn_mock:
                results.append(bench_mock(args.seconds, args.rate, args.width, args.height))
            else:
                results.append(bench_live(args.host, args.port, args.seconds))
        except ConnectionError as exc:
            print(f"live benchmark skipped: {exc}", file=sys.stderr)
    print(format_table(results))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
