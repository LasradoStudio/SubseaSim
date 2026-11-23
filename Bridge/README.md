# SubseaSim Bridge

Python side of SubseaSim: clients for the point-cloud stream that the Unreal plugin serves
over TCP (see [`../Docs/PROTOCOL.md`](../Docs/PROTOCOL.md)), plus a mock simulator so the
whole pipeline runs without Unreal.

| Module | What it does |
|---|---|
| `subsea_bridge.protocol` | Wire format encode/decode (zero-copy numpy point records), `StreamParser` (resyncs on `SSIM`), `StreamClient` (auto-reconnecting TCP client) |
| `subsea_bridge.mock_sim` | Stand-in TCP **server**: seabed + pipeline + boulders, ROV flying a racetrack along the pipe, depth-camera cloud, transforms, telemetry |
| `subsea_bridge.viewer_rerun` | Live Rerun view: sensor-frame cloud on the moving vehicle, accumulated voxel map, trajectory, telemetry plots; `--save` for headless `.rrd` |
| `subsea_bridge.recorder` | Raw stream capture (`.ssim`) and world-frame map export to binary `.ply` / `.pcd` (numpy only) |
| `subsea_bridge.ros2_bridge` | `rclpy` node: `PointCloud2`, TF, `Odometry`, telemetry string, for RViz2 |
| `subsea_bridge.pointcloud2` | ROS-free PointCloud2 byte packing used by the bridge (unit tested) |
| `subsea_bridge.benchmark` | Decode throughput and latency table |
| `subsea_bridge.geometry`, `subsea_bridge.mapping` | Transforms, TF tree, voxel downsampling, bounded voxel map |

Conventions everywhere: REP-103 (metres, radians, X forward, Y left, Z up), quaternions x, y, z, w.

## Install

Python >= 3.10. Only numpy is required; Rerun and pytest are extras; ROS is needed only by `ros2_bridge`.

```powershell
cd Bridge
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -e ".[viewer,dev]"
```

All commands below assume the venv's Python (`.\.venv\Scripts\python.exe`, written `python` for short).
Every tool defaults to `127.0.0.1:9870`, the Unreal plugin's default port, and takes `--host` / `--port`.

## Mock simulator

```powershell
python -m subsea_bridge.mock_sim                      # serve on 127.0.0.1:9870, 320x240 @ 10 Hz
python -m subsea_bridge.mock_sim --port 9871 --rate 15 --width 160 --height 120 --duration 60
```

Flags: `--host` (default `127.0.0.1`; use `0.0.0.0` to serve Docker or another machine, which may
raise a Windows Firewall prompt), `--port` (0 = ephemeral), `--rate`, `--width`, `--height`, `--hfov`,
`--max-range`, `--turbidity`, `--speed`, `--seed`, `--duration` (0 = forever).

Per tick it sends `map -> base_link`, `base_link -> depth_cam`, the cloud, then telemetry, all with
the same stamp. `map` has the sea surface at z = 0, so the vehicle sits near z = -31 and
`depth_m = -z`. It honours the optional command frames `set_turbidity` and `set_mode`
(`station_keep` stops the vehicle). Rendering 320x240 costs roughly 50-70 ms per cloud in numpy, so
10 Hz is the practical default at full resolution; lower the resolution for higher rates.

## Rerun viewer

```powershell
python -m subsea_bridge.viewer_rerun                          # spawns the Rerun viewer window
python -m subsea_bridge.viewer_rerun --save out.rrd --duration 10   # headless, no window
python -m subsea_bridge.viewer_rerun --connect rerun+http://127.0.0.1:9876/proxy
rerun out.rrd                                                 # open a recording later
```

Flags: `--save PATH`, `--connect URL`, `--duration` (0 = until Ctrl+C), `--voxel` (0.10 m),
`--max-map-points` (300000), `--map-every` (integrate every Nth cloud, default 2),
`--map-period` (sim seconds between map re-logs, default 1.0), `--quiet`.

Written against **rerun-sdk 0.43** (`set_time`, `Scalars`, `SeriesLines`, `TransformAxes3D`).
Transforms are logged on the entity hierarchy (`world/base_link/depth_cam/points`), so the live cloud
rides on the vehicle; the map (`world/map`) is voxel-downsampled with the oldest points evicted past
the cap, and re-logged whole once per `--map-period`. Recordings grow by about the raw cloud rate
(~9 MB/s for the default mock), dominated by the live cloud. The client reconnects automatically if
the sim restarts.

## Recorder / map export

```powershell
python -m subsea_bridge.recorder --out run.ssim --duration 30 --ply map.ply --pcd map.pcd
python -m subsea_bridge.recorder --input run.ssim --ply map.ply --voxel 0.05     # offline export
```

Flags: `--out` (raw capture), `--input` (read a capture instead of connecting), `--duration`,
`--clouds N`, `--ply`, `--pcd`, `--voxel` (0.05 m), `--max-points` (5 000 000), `--map-every`.
A `.ssim` file is the wire stream verbatim, so `protocol.read_recording()` replays it. PLY is
`binary_little_endian` with `x y z` float, `red green blue` uchar, `intensity` uchar; PCD is
binary v0.7 with `x y z rgb intensity`.

## Benchmark

```powershell
python -m subsea_bridge.benchmark --spawn-mock              # offline + live against a child-process mock
python -m subsea_bridge.benchmark --mode live --seconds 10  # against whatever is on 127.0.0.1:9870
python -m subsea_bridge.benchmark --mode offline --points 307200
```

Offline pushes pre-encoded frames through the parser/decoder (the client's ceiling). Live reports
what a simulator actually delivers. End-to-end latency is shown only if telemetry carries the
optional `wall_time_s` key (Unix seconds at send; the mock adds it) and is only meaningful when the
sim and the benchmark share a clock.

## Tests

```powershell
python -m pytest
```

Covers protocol round-trips against hand-packed bytes, stream resync (garbage, false magics, bad
headers, truncated frames, 1-byte chunking), voxel downsampling and the bounded map, PLY/PCD
writers, PointCloud2 packing, and end-to-end runs against `mock_sim` on an ephemeral port
(N frames of each type, two clients, reconnect after a server restart, commands, recording,
Rerun `.rrd` output).

## ROS 2 bridge

`ros2_bridge.py` targets ROS 2 Jazzy and has been run live against the Unreal sim with RViz2, in
Ubuntu 24.04 under WSL (see the top-level README and `Scripts/wsl/`). The byte layout it publishes
is produced by `pointcloud2.pack_pointcloud2`, which is unit tested. The `Dockerfile` and
`docker-compose.yml` have not been run.

On a machine with ROS 2 Jazzy:

```bash
export PYTHONPATH=/path/to/Bridge:$PYTHONPATH
python3 -m subsea_bridge.ros2_bridge --ros-args -r __ns:=/subsea -p host:=127.0.0.1 -p port:=9870
rviz2 -d docker/rviz/subsea.rviz
```

Topics (relative, so the namespace above gives `/subsea/...`): `points` (`sensor_msgs/PointCloud2`:
`x y z` float32, `rgb` packed PCL-style in a float32 field, `intensity` float32 0-255, 20-byte
points), `odom` (`nav_msgs/Odometry`, frame `map`, child `base_link`, twist in the child frame),
`telemetry` (`std_msgs/String` JSON), `sonar/raw` (`sensor_msgs/Image` mono8, rows = range bins,
columns = beams), `sonar/image` (`sensor_msgs/Image` rgb8 fan, rendered only while subscribed),
`/tf` (`map -> base_link`) and `/tf_static` (`base_link -> depth_cam`, `base_link -> sonar`).
Parameters: `host`, `port` (defaults from `SUBSEA_HOST` / `SUBSEA_PORT`), `map_frame`, `use_sim_stamps`,
`static_mounts` (true: non-map transforms go to `/tf_static`), `sonar_fan_width` (640, 0 = off).

With Docker:

```bash
cd Bridge/docker
docker compose up --build          # bridge + RViz2, sim expected at host.docker.internal:9870
SUBSEA_HOST=192.168.1.20 SUBSEA_PORT=9870 docker compose up
```

The sim must listen on an address the container can reach (the mock needs `--host 0.0.0.0`).
Display notes for X11 / WSLg are at the top of `docker/docker-compose.yml`.

## Protocol notes for the C++ side

Choices made here where `PROTOCOL.md` leaves room; the plugin should match or the doc should say otherwise:

- `depth_cam` is a REP-103 body-style frame (X along the optical axis, Y left, Z up), **not** a
  ROS `*_optical` frame (Z forward).
- `seq` is one counter per `msg_type`, so the two transforms interleave on the same counter.
- Send a tick's transforms before its cloud, with the same `stamp`; clients place a cloud with the
  latest transforms received and do not interpolate.
- Telemetry frames use `frame_id = "base_link"`; command frames use an empty `frame_id` and `stamp = 0`.
- `heading_deg` is treated as yaw in degrees, counter-clockwise from map +X, in [0, 360).
- Frame names are at most 16 ASCII bytes; a 16-byte name has no NUL terminator.
- There is no checksum. The parser accepts a header when magic and version match, `payload_size`
  is at most 64 MiB, and the size is self-consistent for fixed-layout types (transform = 120,
  cloud = 8 + 16 x `num_points`). Unknown `msg_type` values are skipped using `payload_size`.
  If the plugin ever drops data for a slow client it should drop whole frames.
- Intensity is passed through as 0-255.
