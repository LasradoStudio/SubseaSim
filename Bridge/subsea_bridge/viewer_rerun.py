"""Live Rerun visualisation of the SubseaSim stream.

    python -m subsea_bridge.viewer_rerun                    # spawn the Rerun viewer
    python -m subsea_bridge.viewer_rerun --save out.rrd     # headless recording, no window
    python -m subsea_bridge.viewer_rerun --connect rerun+http://127.0.0.1:9876/proxy

Entity layout (transforms are logged on the entity path hierarchy, so the live cloud rides
on the vehicle)::

    world/                         map frame, right-handed Z up
    world/base_link                Transform3D  (map -> base_link)
    world/base_link/depth_cam      Transform3D  (base_link -> depth_cam)
    world/base_link/depth_cam/points   live cloud in the sensor frame
    world/map                      accumulated, voxel-downsampled world-frame map
    world/trajectory               vehicle path
    telemetry/<key>                scalar time series
    telemetry/log                  mode changes and other text

Written against rerun-sdk 0.43 (``set_time``, ``Scalars``, ``SeriesLines``).
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path
from typing import Any

import numpy as np
import numpy.typing as npt
import rerun as rr
import rerun.blueprint as rrb

from .mapping import MapBuilder
from .protocol import DEFAULT_PORT, Message, PointCloud, SonarImage, StreamClient, Telemetry, Transform
from .sonar import render_fan

TIMELINE = "sim_time"
WORLD = "world"
#: Telemetry keys that get their own plot in the default layout, with a title.
PLOTS: tuple[tuple[str, str], ...] = (
    ("depth_m", "Depth [m]"),
    ("altitude_m", "Altitude [m]"),
    ("speed_mps", "Speed [m/s]"),
    ("thrusters", "Thrusters"),
    ("sensor_ms", "Sensor time [ms]"),
    ("points", "Points per cloud"),
)

_RAMP = np.array(
    [[68, 1, 84], [59, 82, 139], [33, 145, 140], [94, 201, 98], [253, 231, 37]], dtype=np.float32
)


def colorize(values: npt.NDArray[np.floating[Any]], lo: float, hi: float) -> npt.NDArray[np.uint8]:
    """Map scalars to a viridis-like colour ramp (used when a cloud carries no rgb)."""
    t = np.clip((values.astype(np.float32) - lo) / max(hi - lo, 1e-6), 0.0, 1.0) * (len(_RAMP) - 1)
    i = np.minimum(t.astype(np.intp), len(_RAMP) - 2)
    f = (t - i)[:, None]
    return (_RAMP[i] * (1.0 - f) + _RAMP[i + 1] * f).astype(np.uint8)


def make_blueprint() -> rrb.Blueprint:
    plots = [
        rrb.TimeSeriesView(origin=f"/telemetry/{key}", name=title) for key, title in PLOTS
    ]
    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial3DView(origin=f"/{WORLD}", name="Subsea scene", line_grid=False),
            rrb.Vertical(
                rrb.Spatial2DView(origin="/sonar", name="Imaging sonar"),
                rrb.Grid(*plots, grid_columns=2),
                rrb.TextLogView(origin="/telemetry/log", name="Events"),
                row_shares=[3, 4, 1],
            ),
            column_shares=[3, 2],
        ),
        collapse_panels=True,
    )


class RerunLogger:
    """Turns decoded stream messages into Rerun log calls."""

    def __init__(
        self,
        rec: rr.RecordingStream,
        *,
        voxel: float = 0.10,
        max_map_points: int = 300_000,
        map_every: int = 2,
        map_period: float = 1.0,
        world_frame: str = "map",
    ) -> None:
        self.rec = rec
        self.builder = MapBuilder(voxel, max_map_points, world_frame, every=map_every)
        self.world_frame = world_frame
        self.map_period = map_period
        self._last_map_log = -float("inf")
        self._map_dirty = False
        self._trajectory: list[npt.NDArray[np.float64]] = []
        self._traj_stride = 1
        self._traj_count = 0
        self._axes_logged: set[str] = set()
        self._styled: set[str] = set()
        self._last_mode: str | None = None
        self._last_stamp = 0.0
        self.counts = {"cloud": 0, "transform": 0, "telemetry": 0, "sonar": 0}
        rec.log(WORLD, rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)

    # -- helpers -----------------------------------------------------------------

    def _frame_path(self, frame: str) -> str | None:
        """Entity path of a TF frame, or None if its chain to the world frame is unknown."""
        chain = self.builder.tf.chain(frame)
        if chain[0] != self.world_frame:
            return None
        return "/".join([WORLD, *chain[1:]])

    def _set_time(self, stamp: float) -> None:
        self._last_stamp = stamp
        self.rec.set_time(TIMELINE, duration=stamp)

    # -- message handlers ----------------------------------------------------------

    def handle(self, msg: Message) -> None:
        if isinstance(msg, Transform):
            self._on_transform(msg)
        elif isinstance(msg, PointCloud):
            self._on_cloud(msg)
        elif isinstance(msg, Telemetry):
            self._on_telemetry(msg)
        elif isinstance(msg, SonarImage):
            self._on_sonar(msg)

    def _on_sonar(self, msg: SonarImage) -> None:
        self.counts["sonar"] += 1
        self._set_time(msg.header.stamp)
        self.rec.log("sonar", rr.Image(render_fan(msg, width=640)))

    def _on_transform(self, msg: Transform) -> None:
        self.counts["transform"] += 1
        self.builder.process(msg)
        path = self._frame_path(msg.child_frame)
        if path is None:
            return
        self._set_time(msg.header.stamp)
        self.rec.log(
            path,
            rr.Transform3D(translation=msg.position, quaternion=rr.Quaternion(xyzw=msg.orientation)),
        )
        if path not in self._axes_logged:
            self._axes_logged.add(path)
            self.rec.log(path, rr.TransformAxes3D(0.5), static=True)
        if msg.parent_frame == self.world_frame:
            self._traj_count += 1
            if self._traj_count % self._traj_stride == 0:
                self._trajectory.append(np.array(msg.position, dtype=np.float64))
                if len(self._trajectory) > 4000:  # keep the strip bounded on long runs
                    self._trajectory = self._trajectory[::2]
                    self._traj_stride *= 2
                if len(self._trajectory) >= 2:
                    self.rec.log(
                        f"{WORLD}/trajectory",
                        rr.LineStrips3D(
                            [np.asarray(self._trajectory, dtype=np.float32)],
                            colors=[255, 120, 0],
                            radii=0.02,
                        ),
                    )

    def _on_cloud(self, msg: PointCloud) -> None:
        self.counts["cloud"] += 1
        stamp = msg.header.stamp
        path = self._frame_path(msg.header.frame_id)
        if path is None:
            return  # no transform chain yet; the next tick will have one
        self._set_time(stamp)
        xyz = msg.xyz
        if msg.has_rgb:
            colors: npt.NDArray[np.uint8] = msg.rgb
        elif msg.has_intensity:
            colors = colorize(msg.intensity, 0.0, 255.0)
        else:
            colors = colorize(np.linalg.norm(xyz, axis=1), 0.0, 12.0)
        self.rec.log(f"{path}/points", rr.Points3D(xyz, colors=colors, radii=0.012))

        if self.builder.process(msg):
            self._map_dirty = True
        if self._map_dirty and abs(stamp - self._last_map_log) >= self.map_period:
            vmap = self.builder.map
            self._last_map_log = stamp
            self._map_dirty = False
            self.rec.log(
                f"{WORLD}/map",
                rr.Points3D(vmap.xyz, colors=vmap.rgb, radii=vmap.voxel * 0.5),
            )
            self.rec.log("telemetry/map_points", rr.Scalars(float(len(vmap))))

    def _on_telemetry(self, msg: Telemetry) -> None:
        self.counts["telemetry"] += 1
        self._set_time(msg.header.stamp)
        for key, value in msg.data.items():
            if key == "wall_time_s" or isinstance(value, bool):
                continue
            path = f"telemetry/{key}"
            if isinstance(value, (int, float)):
                self.rec.log(path, rr.Scalars(float(value)))
            elif isinstance(value, list) and value and all(
                isinstance(v, (int, float)) and not isinstance(v, bool) for v in value
            ):
                if path not in self._styled:
                    self._styled.add(path)
                    names = [f"{key}[{i}]" for i in range(len(value))]
                    self.rec.log(path, rr.SeriesLines(names=names), static=True)
                self.rec.log(path, rr.Scalars(np.asarray(value, dtype=np.float64)))
        mode = msg.data.get("mode")
        if isinstance(mode, str) and mode != self._last_mode:
            self._last_mode = mode
            self.rec.log("telemetry/log", rr.TextLog(f"mode -> {mode}", level="INFO"))


def run(
    client: StreamClient,
    logger: RerunLogger,
    duration: float = 0.0,
    status_period: float = 2.0,
    quiet: bool = False,
) -> None:
    """Pump messages from ``client`` into ``logger`` until ``duration`` elapses (0 = forever)."""
    t0 = time.monotonic()
    next_status = t0 + status_period
    was_connected = False
    while duration <= 0 or time.monotonic() - t0 < duration:
        for msg in client.poll(0.25):
            logger.handle(msg)
        now = time.monotonic()
        if client.connected != was_connected and not quiet:
            was_connected = client.connected
            state = "connected to" if was_connected else f"lost ({client.last_error}); retrying"
            print(f"[viewer] {state} {client.host}:{client.port}", flush=True)
        if now >= next_status and not quiet:
            next_status = now + status_period
            c = logger.counts
            print(
                f"[viewer] t={logger._last_stamp:8.2f}s clouds={c['cloud']} tf={c['transform']} "
                f"telemetry={c['telemetry']} map={len(logger.builder.map)} pts "
                f"rx={client.bytes_received / 1e6:.1f} MB resyncs={client.parser.resyncs}",
                flush=True,
            )


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Visualise the SubseaSim stream in Rerun")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    out = ap.add_mutually_exclusive_group()
    out.add_argument("--save", type=Path, help="write an .rrd recording instead of opening a viewer (headless)")
    out.add_argument("--connect", metavar="URL", help="send to an already running Rerun viewer (gRPC URL)")
    ap.add_argument("--duration", type=float, default=0.0, help="stop after this many seconds (0 = until Ctrl+C)")
    ap.add_argument("--voxel", type=float, default=0.10, help="map voxel size, metres")
    ap.add_argument("--max-map-points", type=int, default=300_000, help="cap on accumulated map points")
    ap.add_argument("--map-every", type=int, default=2, help="integrate every Nth cloud into the map")
    ap.add_argument("--map-period", type=float, default=1.0, help="sim seconds between map re-logs")
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args(argv)

    rec = rr.RecordingStream("subsea_sim")
    blueprint = make_blueprint()
    if args.save is not None:
        args.save.parent.mkdir(parents=True, exist_ok=True)
        rec.save(str(args.save), default_blueprint=blueprint)
    elif args.connect:
        rec.connect_grpc(args.connect, default_blueprint=blueprint)
    else:
        rec.spawn(default_blueprint=blueprint)

    logger = RerunLogger(
        rec,
        voxel=args.voxel,
        max_map_points=args.max_map_points,
        map_every=args.map_every,
        map_period=args.map_period,
    )
    try:
        with StreamClient(args.host, args.port) as client:
            run(client, logger, args.duration, quiet=args.quiet)
    except KeyboardInterrupt:
        pass
    finally:
        rec.flush()
        rec.disconnect()
    if args.save is not None and not args.quiet:
        size = args.save.stat().st_size if args.save.exists() else 0
        print(f"[viewer] wrote {args.save} ({size / 1e6:.1f} MB), counts {logger.counts}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
