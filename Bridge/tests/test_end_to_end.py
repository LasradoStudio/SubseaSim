from __future__ import annotations

import math
import time
from pathlib import Path

import numpy as np
import pytest

from subsea_bridge import protocol as p
from subsea_bridge.benchmark import bench_live, bench_offline, format_table
from subsea_bridge.geometry import TransformTree, transform_points
from subsea_bridge.mapping import MapBuilder
from subsea_bridge.mock_sim import MockSim, MockSimConfig, path_xy, pipe_y
from subsea_bridge.recorder import record

N = 5  # frames of each type to collect


def _collect(client: p.StreamClient, want: int, timeout: float = 20.0) -> dict[str, list[p.Message]]:
    got: dict[str, list[p.Message]] = {"cloud": [], "tf_base": [], "tf_cam": [], "telemetry": []}
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and min(len(v) for v in got.values()) < want:
        for msg in client.poll(0.2):
            if isinstance(msg, p.PointCloud):
                got["cloud"].append(msg)
            elif isinstance(msg, p.Transform):
                got["tf_base" if msg.child_frame == "base_link" else "tf_cam"].append(msg)
            elif isinstance(msg, p.Telemetry):
                got["telemetry"].append(msg)
    return got


def test_mock_sim_step_is_well_formed(small_sim_config: MockSimConfig) -> None:
    sim = MockSim(small_sim_config)
    msgs = [p.decode_frame(f) for f in sim.step()]
    tf_base, tf_cam, cloud, tel = msgs
    assert isinstance(tf_base, p.Transform) and (tf_base.parent_frame, tf_base.child_frame) == ("map", "base_link")
    assert isinstance(tf_cam, p.Transform) and (tf_cam.parent_frame, tf_cam.child_frame) == ("base_link", "depth_cam")
    assert isinstance(cloud, p.PointCloud) and cloud.header.frame_id == "depth_cam"
    assert isinstance(tel, p.Telemetry)
    assert len({m.header.stamp for m in msgs}) == 1  # one stamp per tick
    assert np.isclose(np.linalg.norm(tf_base.orientation), 1.0)


def test_mock_scene_geometry_is_plausible(small_sim_config: MockSimConfig) -> None:
    cfg = small_sim_config
    sim = MockSim(cfg)
    tree = TransformTree()
    world: list[np.ndarray] = []
    for _ in range(3):
        tf_base, tf_cam, cloud, tel = (p.decode_frame(f) for f in sim.step())
        assert isinstance(tf_base, p.Transform) and isinstance(tf_cam, p.Transform)
        assert isinstance(cloud, p.PointCloud) and isinstance(tel, p.Telemetry)
        tree.update(tf_base)
        tree.update(tf_cam)

        # Sensor frame: X forward (optical axis), within range and field of view.
        xyz = cloud.xyz
        rng = np.linalg.norm(xyz, axis=1)
        assert cloud.num_points > 0.3 * cfg.width * cfg.height
        assert cloud.num_points < cfg.width * cfg.height  # dropout + out-of-range rays
        assert np.all(xyz[:, 0] > 0) and rng.max() <= cfg.max_range + 1e-3 and rng.min() >= cfg.min_range - 1e-3
        half = math.tan(math.radians(cfg.hfov_deg) / 2)
        assert np.all(np.abs(xyz[:, 1]) <= xyz[:, 0] * half * 1.001 + 1e-4)

        # World frame: every return lies on the seabed/pipe/boulders, a couple of metres
        # below the vehicle, never above it.
        T = tree.lookup("depth_cam", "map")
        assert T is not None
        w = transform_points(T, xyz)
        world.append(w)
        veh_z = float(tf_base.position[2])
        assert np.all(w[:, 2] < veh_z - 0.3)
        assert abs(np.median(w[:, 2]) - (-cfg.seabed_depth_m)) < 1.5

        # Telemetry agrees with the transform.
        d = tel.data
        assert d["depth_m"] == pytest.approx(-veh_z, abs=1e-3)
        assert 1.0 < d["altitude_m"] < 3.0
        assert d["points"] == cloud.num_points
        assert len(d["thrusters"]) == 8 and all(-1.0 <= t <= 1.0 for t in d["thrusters"])
        assert d["speed_mps"] == pytest.approx(cfg.speed_mps, rel=0.2)
        assert d["mode"] == "pipeline_follow"
        # Vehicle is moving forward in its own frame (X forward) with negligible sway.
        assert tf_base.linear_vel[0] == pytest.approx(cfg.speed_mps, rel=0.1)
        assert abs(tf_base.linear_vel[1]) < 0.05

    # The track runs parallel to the pipe, one lane offset to its left.
    x, y, _ = path_xy(0.0)
    assert y - float(pipe_y(x)) == pytest.approx(0.9)


def test_end_to_end_reads_n_frames_of_each_type(small_sim_config: MockSimConfig) -> None:
    with MockSim(small_sim_config) as sim:
        assert sim.port != 0
        with p.StreamClient("127.0.0.1", sim.port) as client:
            got = _collect(client, N)
            assert client.parser.resyncs == 0 and client.decode_errors == 0
        for name, msgs in got.items():
            assert len(msgs) >= N, f"only {len(msgs)} {name} frames"

        clouds = [m for m in got["cloud"] if isinstance(m, p.PointCloud)]
        seqs = [c.header.seq for c in clouds]
        assert seqs == list(range(seqs[0], seqs[0] + len(seqs)))  # no gaps, in order
        stamps = [c.header.stamp for c in clouds]
        assert np.allclose(np.diff(stamps), 1.0 / small_sim_config.rate_hz)
        assert all(c.num_points > 0 and c.has_rgb and c.has_intensity for c in clouds)

        tf_seqs = sorted(m.header.seq for m in got["tf_base"] + got["tf_cam"])
        assert len(set(tf_seqs)) == len(tf_seqs)  # the two transforms share one counter
        tel = got["telemetry"][0]
        assert isinstance(tel, p.Telemetry)
        for key in ("depth_m", "altitude_m", "heading_deg", "speed_mps", "thrusters", "mode", "waypoint",
                    "turbidity", "sensor_ms", "points", "fps"):
            assert key in tel.data


def test_two_clients_receive_the_same_stream(small_sim_config: MockSimConfig) -> None:
    with MockSim(small_sim_config) as sim:
        with p.StreamClient("127.0.0.1", sim.port) as a, p.StreamClient("127.0.0.1", sim.port) as b:
            a.poll(0.5)
            b.poll(0.5)
            ga, gb = _collect(a, 3), _collect(b, 3)
        sa = {c.header.seq: c for c in ga["cloud"] if isinstance(c, p.PointCloud)}
        sb = {c.header.seq: c for c in gb["cloud"] if isinstance(c, p.PointCloud)}
        common = set(sa) & set(sb)
        assert common
        for seq in common:
            assert np.array_equal(sa[seq].points, sb[seq].points)


def test_client_reconnects_after_server_restart(small_sim_config: MockSimConfig) -> None:
    sim = MockSim(small_sim_config).start()
    port = sim.port
    client = p.StreamClient("127.0.0.1", port, reconnect_delay=0.05, max_reconnect_delay=0.2)
    try:
        assert len(_collect(client, 2)["cloud"]) >= 2
        sim.stop()
        deadline = time.monotonic() + 10.0
        while client.connected and time.monotonic() < deadline:
            client.poll(0.1)
        assert not client.connected and client.last_error

        cfg2 = MockSimConfig(**{**small_sim_config.__dict__, "port": port})
        sim = MockSim(cfg2).start()
        got = _collect(client, 2, timeout=30.0)
        assert len(got["cloud"]) >= 2 and client.connects == 2
    finally:
        client.close()
        sim.stop()


def test_client_without_reconnect_raises() -> None:
    import socket

    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
    client = p.StreamClient("127.0.0.1", port, reconnect=False, connect_timeout=3.0)
    with pytest.raises(ConnectionError):
        client.poll(0.1)
    client.close()


def test_commands_reach_the_sim(small_sim_config: MockSimConfig) -> None:
    with MockSim(small_sim_config) as sim:
        with p.StreamClient("127.0.0.1", sim.port) as client:
            _collect(client, 1)
            assert client.send_command({"cmd": "set_turbidity", "value": 0.4})
            assert client.send_command({"cmd": "set_mode", "mode": "station_keep"})
            assert client.send_command({"cmd": "not_a_real_command"})
            deadline = time.monotonic() + 10.0
            seen = False
            while time.monotonic() < deadline and not seen:
                for msg in client.poll(0.2):
                    if isinstance(msg, p.Telemetry) and msg.data["mode"] == "station_keep":
                        assert msg.data["turbidity"] == pytest.approx(0.4)
                        assert msg.data["speed_mps"] < 0.1
                        seen = True
            assert seen
        assert len(sim.commands) == 3


def test_record_live_stream_and_build_map(small_sim_config: MockSimConfig, tmp_path: Path) -> None:
    cap = tmp_path / "live.ssim"
    builder = MapBuilder(voxel=0.2)
    with MockSim(small_sim_config) as sim, p.StreamClient("127.0.0.1", sim.port) as client:
        stats = record(client, cap, max_clouds=N, builder=builder, idle_timeout=15.0)
    assert stats.by_type.get(1, 0) >= N
    assert cap.stat().st_size == stats.bytes
    replay = [p.decode(f) for f in p.read_recording(cap)]
    assert sum(isinstance(m, p.PointCloud) for m in replay) == stats.by_type[1]
    assert builder.clouds_integrated >= N - 1 and len(builder.map) > 100
    # The mapped points sit on the seabed in the map frame.
    assert abs(float(np.median(builder.map.xyz[:, 2])) + small_sim_config.seabed_depth_m) < 1.5


def test_benchmark_smoke(small_sim_config: MockSimConfig) -> None:
    off = bench_offline(num_points=2000, ticks=20)
    assert off.clouds == 20 and off.points == 40_000 and off.frames == 80 and off.resyncs == 0
    with MockSim(small_sim_config) as sim:
        live = bench_live("127.0.0.1", sim.port, seconds=1.0, warmup=0.2)
    assert live.clouds > 5 and live.latency_ms and live.interarrival_ms
    table = format_table([off, live])
    assert "point clouds" in table and "end-to-end latency" in table


def test_rerun_logger_writes_rrd(small_sim_config: MockSimConfig, tmp_path: Path) -> None:
    rr = pytest.importorskip("rerun")
    from subsea_bridge.viewer_rerun import RerunLogger, make_blueprint

    out = tmp_path / "test.rrd"
    rec = rr.RecordingStream("subsea_sim_test")
    rec.save(str(out), default_blueprint=make_blueprint())
    logger = RerunLogger(rec, voxel=0.2, map_every=1, map_period=0.0)
    sim = MockSim(small_sim_config)
    for _ in range(5):
        for frame in sim.step():
            logger.handle(p.decode_frame(frame))
    rec.flush()
    rec.disconnect()
    assert logger.counts == {"cloud": 5, "transform": 10, "telemetry": 5, "sonar": 0}
    assert len(logger.builder.map) > 0
    assert out.stat().st_size > 10_000
