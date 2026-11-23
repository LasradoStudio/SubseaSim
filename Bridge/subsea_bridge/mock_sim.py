"""Stand-in for the Unreal SubseaSim plugin: a TCP server speaking the same wire protocol.

Synthesises an undulating seabed with a pipeline and boulders, flies a virtual ROV along
the pipeline (racetrack: out along one side, back along the other) and emits, per tick:

1. Transform ``map -> base_link`` (with body-frame twist)
2. Transform ``base_link -> depth_cam``
3. PointCloud in ``depth_cam`` (X = optical axis, Y left, Z up)
4. Telemetry JSON

``map`` has Z up with the sea surface at z = 0, so vehicle depth is ``-z``.

Run:  python -m subsea_bridge.mock_sim --port 9870
"""

from __future__ import annotations

import argparse
import math
import queue
import socket
import sys
import threading
import time
from dataclasses import dataclass
from typing import Any

import numpy as np
import numpy.typing as npt

from .geometry import quat_from_euler, quat_to_matrix
from .protocol import (
    DEFAULT_PORT,
    FLAG_INTENSITY,
    FLAG_RGB,
    POINT_DTYPE,
    Command,
    PointArray,
    ProtocolError,
    StreamParser,
    decode,
    encode_point_cloud,
    encode_telemetry,
    encode_transform,
)

F32 = npt.NDArray[np.float32]


@dataclass
class MockSimConfig:
    host: str = "127.0.0.1"
    port: int = DEFAULT_PORT
    rate_hz: float = 10.0
    width: int = 320
    height: int = 240
    hfov_deg: float = 90.0
    min_range: float = 0.25
    max_range: float = 12.0
    turbidity: float = 0.15
    speed_mps: float = 0.5
    altitude_m: float = 2.0
    seabed_depth_m: float = 33.0
    grid_res_m: float = 0.03
    cam_pitch_deg: float = 35.0
    seed: int = 7
    #: Render and advance even with no client connected (used by tests calling step()).
    always_render: bool = False


# --------------------------------------------------------------------------------------
# Scene
# --------------------------------------------------------------------------------------

PIPE_LENGTH = 60.0
PIPE_RADIUS = 0.25
PIPE_WAVELENGTH = 40.0
PIPE_AMPLITUDE = 1.2
LANE_OFFSET = 0.9
RUN_X0 = 10.0  # the pipe centreline has zero lateral slope at both run ends
RUN_X1 = 50.0

CLASS_SEABED, CLASS_PIPE, CLASS_BOULDER = 0, 1, 2
_PALETTE = np.array([[196, 178, 138], [222, 184, 48], [118, 116, 110]], dtype=np.float32)
_REFLECTIVITY = np.array([0.75, 1.0, 0.6], dtype=np.float32)
_ATTEN_RGB = np.array([0.20, 0.055, 0.04], dtype=np.float32)  # per metre, clear water


def pipe_y(x: Any) -> Any:
    return PIPE_AMPLITUDE * np.sin(2.0 * np.pi * x / PIPE_WAVELENGTH)


def seabed_z(x: Any, y: Any, depth: float) -> Any:
    """Smooth analytic seabed height (map frame, Z up, surface at 0)."""
    return (
        -depth
        + 0.35 * np.sin(0.21 * x + 0.5) * np.cos(0.17 * y)
        + 0.12 * np.sin(0.63 * x + 0.65 * y)
        + 0.04 * np.sin(1.7 * x) * np.sin(2.1 * y)
        + 0.008 * x
    )


class Scene:
    """Heightfield (seabed + pipe + boulders) rasterised on a regular grid."""

    def __init__(self, cfg: MockSimConfig) -> None:
        rng = np.random.default_rng(cfg.seed)
        res = cfg.grid_res_m
        margin = cfg.max_range + 3.0
        self.x0 = RUN_X0 - LANE_OFFSET - margin
        self.y0 = -(PIPE_AMPLITUDE + LANE_OFFSET + margin)
        x1 = RUN_X1 + LANE_OFFSET + margin
        self.nx = int(math.ceil((x1 - self.x0) / res)) + 1
        self.ny = int(math.ceil((-2.0 * self.y0) / res)) + 1
        self.inv_res = np.float32(1.0 / res)
        self.depth = cfg.seabed_depth_m

        xs = (self.x0 + res * np.arange(self.nx, dtype=np.float32))[:, None]
        ys = (self.y0 + res * np.arange(self.ny, dtype=np.float32))[None, :]
        H = seabed_z(xs, ys, self.depth).astype(np.float32)
        H += rng.normal(0.0, 0.006, size=H.shape).astype(np.float32)
        klass = np.zeros(H.shape, dtype=np.uint8)

        # Pipeline, 40 % buried, with a fatter collar every 12 m.
        pc = pipe_y(xs)
        slope = PIPE_AMPLITUDE * (2.0 * np.pi / PIPE_WAVELENGTH) * np.cos(2.0 * np.pi * xs / PIPE_WAVELENGTH)
        d = (ys - pc) / np.sqrt(1.0 + slope * slope)
        radius = np.where(np.abs((xs % 12.0) - 6.0) < 0.18, PIPE_RADIUS * 1.35, PIPE_RADIUS).astype(np.float32)
        zc = seabed_z(xs, pc, self.depth) + 0.2 * PIPE_RADIUS
        on_pipe = (np.abs(d) < radius) & (xs >= 0.0) & (xs <= PIPE_LENGTH)
        top = zc + np.sqrt(np.maximum(radius * radius - d * d, 0.0))
        on_pipe &= top > H
        H = np.where(on_pipe, top, H).astype(np.float32)
        klass[on_pipe] = CLASS_PIPE

        # Boulders: squashed spheres scattered either side of the pipe.
        self.boulders: list[tuple[float, float, float]] = []
        for _ in range(16):
            bx = float(rng.uniform(2.0, PIPE_LENGTH - 2.0))
            side = 1.0 if rng.random() < 0.5 else -1.0
            by = float(pipe_y(bx)) + side * float(rng.uniform(1.6, 6.5))
            br = float(rng.uniform(0.3, 0.95))
            squash = float(rng.uniform(0.6, 0.95))
            self.boulders.append((bx, by, br))
            i0 = max(int((bx - br - self.x0) / res), 0)
            i1 = min(int((bx + br - self.x0) / res) + 2, self.nx)
            j0 = max(int((by - br - self.y0) / res), 0)
            j1 = min(int((by + br - self.y0) / res) + 2, self.ny)
            if i0 >= i1 or j0 >= j1:
                continue
            lx, ly = xs[i0:i1], ys[:, j0:j1]
            r2 = br * br - (lx - bx) ** 2 - (ly - by) ** 2
            bz = float(seabed_z(bx, by, self.depth)) - 0.25 * br
            btop = (bz + squash * np.sqrt(np.maximum(r2, 0.0))).astype(np.float32)
            sub = H[i0:i1, j0:j1]
            hit = (r2 > 0.0) & (btop > sub)
            sub[hit] = btop[hit]
            klass[i0:i1, j0:j1][hit] = CLASS_BOULDER

        # World-fixed albedo variation so colours do not swim between frames.
        shade = 0.82 + 0.10 * np.sin(1.3 * xs + 0.4) * np.sin(0.9 * ys) + rng.normal(0.0, 0.05, size=H.shape)
        self._h = np.ascontiguousarray(H).ravel()
        self._klass = klass.ravel()
        self._shade = np.clip(shade, 0.5, 1.0).astype(np.float32).ravel()

    def index(self, x: F32, y: F32) -> npt.NDArray[np.intp]:
        ix = ((x - np.float32(self.x0)) * self.inv_res + np.float32(0.5)).astype(np.intp)
        iy = ((y - np.float32(self.y0)) * self.inv_res + np.float32(0.5)).astype(np.intp)
        np.clip(ix, 0, self.nx - 1, out=ix)
        np.clip(iy, 0, self.ny - 1, out=iy)
        ix *= self.ny
        ix += iy
        return ix

    def height_at(self, x: float, y: float) -> float:
        idx = self.index(np.array([x], dtype=np.float32), np.array([y], dtype=np.float32))
        return float(self._h[idx[0]])

    def raycast(
        self, origin: npt.NDArray[np.float64], dirs: F32, min_range: float, max_range: float, iters: int = 24
    ) -> tuple[F32, npt.NDArray[np.bool_]]:
        """Range along each unit ray to the heightfield, and a hit mask.

        Relaxed sphere-tracing until a ray first dips below the surface, then bisection
        between the last point above and the first point below.
        """
        ox, oy, oz = (np.float32(v) for v in origin)
        dx = np.ascontiguousarray(dirs[:, 0])
        dy = np.ascontiguousarray(dirs[:, 1])
        dz = np.ascontiguousarray(dirs[:, 2])
        gain = np.float32(0.7) / np.maximum(-dz, np.float32(0.3))
        t = np.full(dx.shape, min_range, dtype=np.float32)
        lo = t.copy()
        hi = np.full(dx.shape, np.inf, dtype=np.float32)
        t_cap = np.float32(max_range + 1.0)
        h = self._h
        for _ in range(iters):
            g = oz + dz * t - h[self.index(ox + dx * t, oy + dy * t)]
            inside = g < 0.0
            np.copyto(hi, t, where=inside)
            np.copyto(lo, t, where=~inside)
            step = np.clip(g * gain, np.float32(0.03), np.float32(1.5))
            t = np.where(np.isfinite(hi), np.float32(0.5) * (lo + hi), np.minimum(t + step, t_cap))
        hit = np.isfinite(hi) & (hi <= max_range) & (hi > min_range)
        return np.float32(0.5) * (lo + hi), hit


# --------------------------------------------------------------------------------------
# Vehicle path
# --------------------------------------------------------------------------------------

_RUN = RUN_X1 - RUN_X0
_TURN = math.pi * LANE_OFFSET
LAP_LENGTH = 2.0 * (_RUN + _TURN)


def path_xy(s: float) -> tuple[float, float, str]:
    """Racetrack around the pipe, parameterised (approximately) by arc length."""
    s = s % LAP_LENGTH
    if s < _RUN:
        x = RUN_X0 + s
        return x, float(pipe_y(x)) + LANE_OFFSET, "pipeline_follow"
    s -= _RUN
    if s < _TURN:
        phi = s / LANE_OFFSET
        return RUN_X1 + LANE_OFFSET * math.sin(phi), float(pipe_y(RUN_X1)) + LANE_OFFSET * math.cos(phi), "turn"
    s -= _TURN
    if s < _RUN:
        x = RUN_X1 - s
        return x, float(pipe_y(x)) - LANE_OFFSET, "pipeline_follow"
    s -= _RUN
    phi = s / LANE_OFFSET
    return RUN_X0 - LANE_OFFSET * math.sin(phi), float(pipe_y(RUN_X0)) - LANE_OFFSET * math.cos(phi), "turn"


# --------------------------------------------------------------------------------------
# Simulator
# --------------------------------------------------------------------------------------


class _Client:
    def __init__(self, sock: socket.socket, addr: Any) -> None:
        self.sock = sock
        self.addr = addr
        self.queue: "queue.Queue[bytes | None]" = queue.Queue(maxsize=4)
        self.alive = True
        self.dropped = 0

    def offer(self, blob: bytes) -> None:
        while True:
            try:
                self.queue.put_nowait(blob)
                return
            except queue.Full:
                try:
                    self.queue.get_nowait()
                    self.dropped += 1
                except queue.Empty:
                    pass

    def close(self) -> None:
        self.alive = False
        try:
            self.sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        try:
            self.sock.close()
        except OSError:
            pass
        try:
            self.queue.put_nowait(None)
        except queue.Full:
            pass


class MockSim:
    """Threaded mock server. ``start()`` binds and serves; ``step()`` renders one tick."""

    def __init__(self, config: MockSimConfig | None = None) -> None:
        self.cfg = config or MockSimConfig()
        cfg = self.cfg
        self.scene = Scene(cfg)
        self._rng = np.random.default_rng(cfg.seed + 1)
        self._dt = 1.0 / cfg.rate_hz
        self._tick = 0
        self._path_s = 0.0
        self._seq = {"cloud": 0, "tf": 0, "telemetry": 0}
        self.turbidity = cfg.turbidity
        self.mode = "pipeline_follow"
        self._hold = False
        self._sensor_ms = 0.0
        self._fps = cfg.rate_hz
        self.commands: list[dict[str, Any]] = []

        # Unit ray directions in depth_cam (X forward, Y left, Z up), row-major from top-left.
        fx = (cfg.width * 0.5) / math.tan(math.radians(cfg.hfov_deg) * 0.5)
        u = (np.arange(cfg.width, dtype=np.float32) + 0.5 - cfg.width * 0.5) / fx
        v = (np.arange(cfg.height, dtype=np.float32) + 0.5 - cfg.height * 0.5) / fx
        uu, vv = np.meshgrid(u, v)
        dirs = np.stack((np.ones_like(uu), -uu, -vv), axis=-1).reshape(-1, 3)
        self._dirs_cam: F32 = (dirs / np.linalg.norm(dirs, axis=1, keepdims=True)).astype(np.float32)

        self._cam_pos = np.array([0.40, 0.0, -0.18])
        self._cam_quat = quat_from_euler(0.0, math.radians(cfg.cam_pitch_deg), 0.0)
        self._cam_rot = quat_to_matrix(self._cam_quat)

        self._server: socket.socket | None = None
        self._threads: list[threading.Thread] = []
        self._clients: list[_Client] = []
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self.port = cfg.port

    # -- kinematics ------------------------------------------------------------

    def _pose(self, s: float, t: float) -> tuple[npt.NDArray[np.float64], tuple[float, float, float]]:
        x, y, _ = path_xy(s)
        xa, ya, _ = path_xy(s - 0.05)
        xb, yb, _ = path_xy(s + 0.05)
        yaw = math.atan2(yb - ya, xb - xa)
        z = float(seabed_z(x, y, self.cfg.seabed_depth_m)) + self.cfg.altitude_m + 0.08 * math.sin(0.5 * t)
        roll = 0.03 * math.sin(0.9 * t)
        pitch = 0.04 * math.sin(0.6 * t + 1.0)
        return np.array([x, y, z]), (roll, pitch, yaw)

    def _state(self, s: float, t: float, moving: bool) -> dict[str, Any]:
        h = 0.02
        ds = self.cfg.speed_mps * h if moving else 0.0
        p, rpy = self._pose(s, t)
        p0, rpy0 = self._pose(s - ds, t - h)
        p1, rpy1 = self._pose(s + ds, t + h)
        q = quat_from_euler(*rpy)
        R = quat_to_matrix(q)
        R0 = quat_to_matrix(quat_from_euler(*rpy0))
        R1 = quat_to_matrix(quat_from_euler(*rpy1))
        lin = R.T @ ((p1 - p0) / (2 * h))
        W = R.T @ ((R1 - R0) / (2 * h))
        ang = np.array([W[2, 1], W[0, 2], W[1, 0]])
        return {"p": p, "q": q, "R": R, "rpy": rpy, "lin": lin, "ang": ang}

    # -- sensor ----------------------------------------------------------------

    def render(self, cam_pos: npt.NDArray[np.float64], cam_rot: npt.NDArray[np.float64]) -> PointArray:
        """Depth-camera point cloud in the camera frame for a camera pose given in map."""
        cfg = self.cfg
        rng = self._rng
        dirs_world = self._dirs_cam @ cam_rot.T.astype(np.float32)
        rng_m, hit = self.scene.raycast(cam_pos, dirs_world, cfg.min_range, cfg.max_range)

        # Range-dependent dropout (worse in turbid water) and range noise.
        rel = rng_m / np.float32(cfg.max_range)
        p_drop = 0.02 + np.minimum(0.9, (0.25 + 1.5 * self.turbidity) * rel**3)
        keep = hit & (rng.random(rng_m.shape, dtype=np.float32) >= p_drop)
        idx = np.flatnonzero(keep)
        r_true = rng_m[idx]
        d_world = dirs_world[idx]
        sigma = 0.004 + 0.0008 * r_true * r_true
        r_meas = r_true + sigma * rng.standard_normal(r_true.shape, dtype=np.float32)
        np.clip(r_meas, cfg.min_range, cfg.max_range, out=r_meas)

        pts = np.empty(idx.shape[0], dtype=POINT_DTYPE)
        xyz = pts.view("<f4").reshape(-1, 4)
        xyz[:, :3] = self._dirs_cam[idx] * r_meas[:, None]

        # Appearance from the true hit location: albedo, then wavelength-dependent attenuation.
        o = cam_pos.astype(np.float32)
        cell = self.scene.index(o[0] + d_world[:, 0] * r_true, o[1] + d_world[:, 1] * r_true)
        klass = self.scene._klass[cell]
        shade = self.scene._shade[cell]
        water = np.float32(1.0 + 4.0 * self.turbidity)
        atten = np.exp(-(r_true[:, None] * water) * _ATTEN_RGB[None, :])
        rgb = _PALETTE[klass] * shade[:, None] * atten + np.array([4.0, 18.0, 26.0], dtype=np.float32) * (1.0 - atten)
        inten = 255.0 * _REFLECTIVITY[klass] * shade * np.exp(-0.09 * water * r_true)
        raw = pts.view(np.uint8).reshape(-1, 16)
        raw[:, 12:15] = np.clip(rgb, 0, 255).astype(np.uint8)
        raw[:, 15] = np.clip(inten, 0, 255).astype(np.uint8)
        return pts

    # -- one tick ----------------------------------------------------------------

    def step(self) -> list[bytes]:
        """Advance one tick and return its encoded frames (2 transforms, cloud, telemetry)."""
        cfg = self.cfg
        stamp = self._tick * self._dt
        moving = not self._hold
        st = self._state(self._path_s, stamp, moving)
        p, R = st["p"], st["R"]
        _, _, path_mode = path_xy(self._path_s)
        mode = "station_keep" if self._hold else (self.mode if self.mode != "pipeline_follow" else path_mode)

        t0 = time.perf_counter()
        cloud = self.render(p + R @ self._cam_pos, R @ self._cam_rot)
        self._sensor_ms = (time.perf_counter() - t0) * 1e3

        u, v, w = (float(x) for x in st["lin"])
        yaw_rate = float(st["ang"][2])
        thr_h = [u - v - 0.6 * yaw_rate, u + v + 0.6 * yaw_rate, u + v - 0.6 * yaw_rate, u - v + 0.6 * yaw_rate]
        roll, pitch, yaw = st["rpy"]
        thr_v = [0.5 + 2.0 * w + s_r * roll * 2.0 + s_p * pitch * 2.0 for s_r, s_p in ((1, 1), (-1, 1), (1, -1), (-1, -1))]
        thrusters = [round(max(-1.0, min(1.0, x)), 3) for x in thr_h + thr_v]

        telemetry = {
            "depth_m": round(-float(p[2]), 3),
            "altitude_m": round(float(p[2]) - self.scene.height_at(float(p[0]), float(p[1])), 3),
            "heading_deg": round(math.degrees(yaw) % 360.0, 2),
            "speed_mps": round(float(np.linalg.norm(st["lin"])), 3),
            "thrusters": thrusters,
            "mode": mode,
            "waypoint": int((self._path_s % LAP_LENGTH) // 5.0),
            "turbidity": round(self.turbidity, 3),
            "sensor_ms": round(self._sensor_ms, 2),
            "points": int(cloud.shape[0]),
            "fps": round(self._fps, 1),
            # Extra key (allowed by the protocol): wall clock at send, lets a client on the
            # same machine measure end-to-end latency.
            "wall_time_s": time.time(),
        }

        seq = self._seq
        frames = [
            encode_transform(seq["tf"], stamp, "base_link", "map", p, st["q"], st["lin"], st["ang"]),
            encode_transform(seq["tf"] + 1, stamp, "depth_cam", "base_link", self._cam_pos, self._cam_quat),
            encode_point_cloud(seq["cloud"], stamp, "depth_cam", cloud, FLAG_RGB | FLAG_INTENSITY),
            encode_telemetry(seq["telemetry"], stamp, telemetry, "base_link"),
        ]
        seq["tf"] += 2
        seq["cloud"] += 1
        seq["telemetry"] += 1
        self._tick += 1
        if moving:
            self._path_s += cfg.speed_mps * self._dt
        return frames

    # -- commands ----------------------------------------------------------------

    def handle_command(self, data: dict[str, Any]) -> None:
        """Apply a client command; unknown commands are ignored, as the protocol requires."""
        self.commands.append(data)
        cmd = data.get("cmd")
        if cmd == "set_turbidity":
            try:
                self.turbidity = max(0.0, min(1.0, float(data.get("value", self.turbidity))))
            except (TypeError, ValueError):
                pass
        elif cmd == "set_mode":
            mode = str(data.get("mode", ""))
            self._hold = mode == "station_keep"
            self.mode = "pipeline_follow" if self._hold else (mode or "pipeline_follow")

    # -- networking --------------------------------------------------------------

    def start(self) -> "MockSim":
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        if sys.platform != "win32":
            srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((self.cfg.host, self.cfg.port))
        srv.listen(8)
        srv.settimeout(0.2)
        self._server = srv
        self.port = int(srv.getsockname()[1])
        self._stop.clear()
        for target in (self._accept_loop, self._produce_loop):
            th = threading.Thread(target=target, name=f"mock-sim-{target.__name__}", daemon=True)
            th.start()
            self._threads.append(th)
        return self

    def stop(self) -> None:
        self._stop.set()
        if self._server is not None:
            try:
                self._server.close()
            except OSError:
                pass
            self._server = None
        with self._lock:
            clients, self._clients = self._clients, []
        for c in clients:
            c.close()
        for th in self._threads:
            th.join(timeout=3.0)
        self._threads.clear()

    def __enter__(self) -> "MockSim":
        return self.start()

    def __exit__(self, *exc: object) -> None:
        self.stop()

    @property
    def num_clients(self) -> int:
        with self._lock:
            return len(self._clients)

    def _accept_loop(self) -> None:
        srv = self._server
        assert srv is not None
        while not self._stop.is_set():
            try:
                sock, addr = srv.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            sock.settimeout(None)
            client = _Client(sock, addr)
            for target in (self._send_loop, self._recv_loop):
                threading.Thread(target=target, args=(client,), name="mock-sim-client", daemon=True).start()
            with self._lock:
                self._clients.append(client)

    def _remove(self, client: _Client) -> None:
        client.close()
        with self._lock:
            if client in self._clients:
                self._clients.remove(client)

    def _send_loop(self, client: _Client) -> None:
        while client.alive and not self._stop.is_set():
            try:
                blob = client.queue.get(timeout=0.5)
            except queue.Empty:
                continue
            if blob is None:
                break
            try:
                client.sock.sendall(blob)
            except OSError:
                break
        self._remove(client)

    def _recv_loop(self, client: _Client) -> None:
        parser = StreamParser(max_payload=1 << 16)
        while client.alive and not self._stop.is_set():
            try:
                data = client.sock.recv(4096)
            except OSError:
                break
            if not data:
                break
            for frame in parser.feed(data):
                try:
                    msg = decode(frame)
                except ProtocolError:
                    continue
                if isinstance(msg, Command):
                    self.handle_command(msg.data)
        self._remove(client)

    def _produce_loop(self) -> None:
        next_tick = time.perf_counter()
        last = next_tick
        while not self._stop.is_set():
            with self._lock:
                clients = list(self._clients)
            if clients or self.cfg.always_render:
                blob = b"".join(self.step())
                for c in clients:
                    c.offer(blob)
                now = time.perf_counter()
                if now > last:
                    self._fps = 0.9 * self._fps + 0.1 * (1.0 / (now - last))
                last = now
            else:
                last = time.perf_counter()
            next_tick += self._dt
            delay = next_tick - time.perf_counter()
            if delay > 0:
                self._stop.wait(delay)
            else:
                next_tick = time.perf_counter()  # running behind: do not accumulate debt


def build_parser() -> argparse.ArgumentParser:
    d = MockSimConfig()
    ap = argparse.ArgumentParser(description="SubseaSim mock simulator (TCP server speaking the SSIM protocol)")
    ap.add_argument("--host", default=d.host, help="bind address (use 0.0.0.0 to serve Docker/other machines)")
    ap.add_argument("--port", type=int, default=d.port, help="TCP port, 0 = ephemeral")
    ap.add_argument("--rate", type=float, default=d.rate_hz, help="ticks per second")
    ap.add_argument("--width", type=int, default=d.width)
    ap.add_argument("--height", type=int, default=d.height)
    ap.add_argument("--hfov", type=float, default=d.hfov_deg, help="horizontal field of view, degrees")
    ap.add_argument("--max-range", type=float, default=d.max_range, help="metres")
    ap.add_argument("--turbidity", type=float, default=d.turbidity, help="0..1")
    ap.add_argument("--speed", type=float, default=d.speed_mps, help="vehicle speed, m/s")
    ap.add_argument("--seed", type=int, default=d.seed)
    ap.add_argument("--duration", type=float, default=0.0, help="exit after this many seconds (0 = run forever)")
    return ap


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    cfg = MockSimConfig(
        host=args.host,
        port=args.port,
        rate_hz=args.rate,
        width=args.width,
        height=args.height,
        hfov_deg=args.hfov,
        max_range=args.max_range,
        turbidity=args.turbidity,
        speed_mps=args.speed,
        seed=args.seed,
    )
    sim = MockSim(cfg).start()
    print(
        f"mock_sim listening on {cfg.host}:{sim.port}  {cfg.width}x{cfg.height} @ {cfg.rate_hz:g} Hz, "
        f"hfov {cfg.hfov_deg:g} deg, max range {cfg.max_range:g} m",
        flush=True,
    )
    deadline = time.monotonic() + args.duration if args.duration > 0 else math.inf
    try:
        while time.monotonic() < deadline:
            time.sleep(min(1.0, max(0.0, deadline - time.monotonic())))
    except KeyboardInterrupt:
        pass
    finally:
        sim.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
