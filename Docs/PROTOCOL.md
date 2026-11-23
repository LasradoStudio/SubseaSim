# SubseaSim wire protocol (v1)

The Unreal plugin is a TCP **server** (default `127.0.0.1:9870`; pass `-SubseaBindAll` to listen on every interface, `-SubseaPort=N` to change the port). Any number of clients may connect;
every client receives the same stream of frames. The stream is one-way (sim -> client) except for
the optional command frame described at the end.

All integers and floats are **little-endian**. All geometry is in **ROS conventions (REP-103)**:
metres, radians, right-handed, X forward, Y left, Z up. The plugin converts from Unreal
(centimetres, left-handed, Y right) before sending, so clients never see Unreal units.

## Frame

| Offset | Type      | Field        | Notes                                              |
|-------:|-----------|--------------|----------------------------------------------------|
| 0      | char[4]   | magic        | ASCII `SSIM`                                       |
| 4      | uint16    | version      | `1`                                                |
| 6      | uint16    | msg_type     | see below                                          |
| 8      | uint32    | seq          | per-message-type counter                           |
| 12     | float64   | stamp        | simulation time, seconds                           |
| 20     | char[16]  | frame_id     | ASCII, NUL padded (e.g. `depth_cam`)               |
| 36     | uint32    | payload_size | bytes that follow                                  |
| 40     | bytes     | payload      |                                                    |

Header is exactly 40 bytes.

## msg_type 1: PointCloud

Points are expressed in the sensor frame named by `frame_id`.

| Type    | Field      | Notes                                         |
|---------|------------|-----------------------------------------------|
| uint32  | num_points |                                               |
| uint32  | flags      | bit0 = rgb valid, bit1 = intensity valid      |
| point[] | points     | `num_points` records of 16 bytes              |

Point record (16 bytes): `float32 x, float32 y, float32 z, uint8 r, uint8 g, uint8 b, uint8 intensity`.

## msg_type 2: Transform

Pose of `frame_id` expressed in `parent_frame`, plus body-frame twist.

| Type       | Field        | Notes                                   |
|------------|--------------|-----------------------------------------|
| char[16]   | parent_frame | ASCII, NUL padded (e.g. `map`)          |
| float64[3] | position     | x, y, z metres                          |
| float64[4] | orientation  | quaternion x, y, z, w                   |
| float64[3] | linear_vel   | m/s, in `frame_id`                      |
| float64[3] | angular_vel  | rad/s, in `frame_id`                    |

Payload is exactly 120 bytes. Frames in use: `map` -> `base_link` (vehicle), `base_link` -> `depth_cam`.

## msg_type 3: Telemetry

UTF-8 JSON object, e.g.

```json
{"depth_m": 31.2, "altitude_m": 2.1, "heading_deg": 87.0, "speed_mps": 0.42,
 "thrusters": [0.1, -0.2, 0.0, 0.3, 0.5, 0.5, 0.5, 0.5],
 "mode": "pipeline_follow", "waypoint": 3, "turbidity": 0.15,
 "sensor_ms": 1.9, "points": 76800, "fps": 60.0}
```

Keys may be added over time; clients must ignore unknown keys.

## msg_type 5: SonarImage

One forward-looking imaging-sonar ping, in the sensor frame named by `frame_id` (`sonar`).

| Type    | Field             | Notes                                                    |
|---------|-------------------|----------------------------------------------------------|
| uint16  | num_beams         |                                                          |
| uint16  | num_bins          |                                                          |
| float32 | hfov              | full horizontal field of view, radians                   |
| float32 | min_range         | metres                                                   |
| float32 | max_range         | metres                                                   |
| float32 | vertical_aperture | full vertical opening of each beam, radians              |
| uint8[] | intensity         | `num_bins` rows x `num_beams` columns, row-major     |

Rows are range bins from `min_range` outwards. Columns are beams from port (left) to starboard,
equally spaced in azimuth: beam *i* points at `hfov/2 - (i + 0.5) * hfov / num_beams` (ROS
convention, positive = left). Intensity is log-compressed echo strength. The mount transform
`base_link -> sonar` is published like the depth camera's.

## msg_type 4: Command (client -> sim, optional)

UTF-8 JSON, same 40-byte header. Example: `{"cmd": "set_mode", "mode": "station_keep"}` or
`{"cmd": "set_turbidity", "value": 0.4}`. The sim ignores commands it does not understand.

| Command | Arguments | Effect |
|---|---|---|
| `set_mode` | `mode`: `manual` / `station_keep` / `pipeline_follow` | autopilot mode |
| `fail_thruster` | `index`, `failed` | fault injection; allocation is re-solved |
| `set_turbidity` | `value` 0..1 | water clarity (fog and depth-camera dropout) |
| `camera` | `shot`: chase / orbit / wide / side / top / fpv / flyby, optional `orbit_dps` | camera shot |
| `lights` | `on` | vehicle lights |
| `hud` | `visible` | pilot HUD on or off |
| `record` | `name`, `frames` | write one PNG per frame to `Saved/Recordings/<name>` (use `-UseFixedTimeStep -FPS=N`) |
| `screenshot` | `name` | single PNG to `Saved/Screenshots` |
| `max_fps` | `value` (0 = uncapped) | frame-rate cap, for frame-rate independence tests |
| `quit` | | clean exit |
