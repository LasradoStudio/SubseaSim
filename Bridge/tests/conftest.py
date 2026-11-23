from __future__ import annotations

import numpy as np
import pytest

from subsea_bridge.mock_sim import MockSimConfig
from subsea_bridge.protocol import (
    PointArray,
    encode_point_cloud,
    encode_telemetry,
    encode_transform,
    make_points,
)


def random_points(n: int, seed: int = 0) -> PointArray:
    rng = np.random.default_rng(seed)
    return make_points(
        rng.uniform(-10, 10, size=(n, 3)),
        rng.integers(0, 256, size=(n, 3)),
        rng.integers(0, 256, size=n),
    )


def tick_frames(seq: int, n: int = 50) -> list[bytes]:
    """The four frames of one sim tick, as separate byte strings."""
    stamp = seq * 0.1
    return [
        encode_transform(2 * seq, stamp, "base_link", "map", (seq, 2.0, -30.0), (0, 0, 0, 1), (0.5, 0, 0), (0, 0, 0.1)),
        encode_transform(2 * seq + 1, stamp, "depth_cam", "base_link", (0.4, 0, -0.2), (0, 0.3007058, 0, 0.9537170)),
        encode_point_cloud(seq, stamp, "depth_cam", random_points(n, seed=seq)),
        encode_telemetry(seq, stamp, {"depth_m": 30.0 + seq, "mode": "pipeline_follow", "thrusters": [0.1] * 8}),
    ]


@pytest.fixture
def small_sim_config() -> MockSimConfig:
    """A mock sim that is cheap to build and render (coarse grid, tiny image)."""
    return MockSimConfig(port=0, rate_hz=40.0, width=64, height=48, grid_res_m=0.1)
