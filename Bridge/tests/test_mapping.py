from __future__ import annotations

import math

import numpy as np
import pytest

from subsea_bridge import protocol as p
from subsea_bridge.geometry import (
    TransformTree,
    pose_matrix,
    quat_from_euler,
    quat_to_matrix,
    transform_points,
    yaw_from_quat,
)
from subsea_bridge.mapping import MapBuilder, VoxelMap, voxel_downsample


def test_quaternion_conventions() -> None:
    # +90 deg yaw turns X (forward) into Y (left)
    R = quat_to_matrix(quat_from_euler(0.0, 0.0, math.pi / 2))
    assert np.allclose(R @ [1, 0, 0], [0, 1, 0], atol=1e-12)
    # positive pitch (about +Y) tips the nose down: X goes towards -Z
    R = quat_to_matrix(quat_from_euler(0.0, math.pi / 2, 0.0))
    assert np.allclose(R @ [1, 0, 0], [0, 0, -1], atol=1e-12)
    # positive roll (about +X) turns Y (left) into Z (up)
    R = quat_to_matrix(quat_from_euler(math.pi / 2, 0.0, 0.0))
    assert np.allclose(R @ [0, 1, 0], [0, 0, 1], atol=1e-12)
    q = quat_from_euler(0.1, -0.2, 0.7)
    assert np.isclose(np.linalg.norm(q), 1.0)
    assert np.isclose(yaw_from_quat(q), 0.7)
    Rq = quat_to_matrix(q)
    assert np.allclose(Rq @ Rq.T, np.eye(3), atol=1e-12) and np.isclose(np.linalg.det(Rq), 1.0)


def test_transform_tree_chain() -> None:
    tree = TransformTree()
    assert tree.lookup("depth_cam", "map") is None
    tree.set("base_link", "depth_cam", pose_matrix([1, 0, 0], quat_from_euler(0, 0, 0)))
    assert tree.lookup("depth_cam", "map") is None
    tree.set("map", "base_link", pose_matrix([10, 5, -30], quat_from_euler(0, 0, math.pi / 2)))
    T = tree.lookup("depth_cam", "map")
    assert T is not None
    # a point 2 m ahead of the camera: 3 m ahead of the vehicle, which faces +Y in map
    assert np.allclose(transform_points(T, [[2.0, 0.0, 0.0]]), [[10.0, 8.0, -30.0]], atol=1e-5)
    assert tree.chain("depth_cam") == ["map", "base_link", "depth_cam"]
    assert np.allclose(tree.lookup("map", "map"), np.eye(4))


def test_voxel_downsample_one_point_per_voxel() -> None:
    rng = np.random.default_rng(0)
    # 4 voxel centres (0.5 m voxels), 100 jittered points each, shuffled
    centres = np.array([[0.25, 0.25, 0.25], [0.75, 0.25, 0.25], [-0.25, 0.25, 0.25], [0.25, 0.25, -3.25]])
    pts = (centres[:, None, :] + rng.uniform(-0.2, 0.2, size=(4, 100, 3))).reshape(-1, 3).astype(np.float32)
    rng.shuffle(pts)
    idx = voxel_downsample(pts, 0.5)
    assert idx.shape == (4,)
    assert np.all(np.diff(idx) > 0)  # ascending, i.e. first-seen points
    cells = np.floor(pts[idx] / 0.5).astype(int)
    assert len({tuple(c) for c in cells}) == 4
    # negative coordinates floor correctly: -0.05 and +0.05 are different voxels
    assert voxel_downsample(np.array([[-0.05, 0, 0], [0.05, 0, 0]], dtype=np.float32), 0.5).shape == (2,)
    # a finer grid keeps more points, a huge one keeps exactly one
    assert voxel_downsample(pts, 0.05).shape[0] > 100
    assert voxel_downsample(pts + np.float32(10.0), 100.0).tolist() == [0]


def test_voxel_downsample_edge_cases() -> None:
    assert voxel_downsample(np.empty((0, 3), dtype=np.float32), 0.1).shape == (0,)
    pts = np.array([[np.nan, 0, 0], [1, 1, 1], [np.inf, 0, 0], [1.01, 1.01, 1.01]], dtype=np.float32)
    assert voxel_downsample(pts, 0.5).tolist() == [1]
    with pytest.raises(ValueError):
        voxel_downsample(pts, 0.0)


def test_voxel_map_deduplicates_across_adds() -> None:
    vmap = VoxelMap(voxel=0.1, max_points=10_000)
    rng = np.random.default_rng(1)
    pts = rng.uniform(0, 2, size=(5000, 3)).astype(np.float32)
    rgb = rng.integers(0, 256, size=(5000, 3), dtype=np.uint8)
    inten = rng.integers(0, 256, size=5000, dtype=np.uint8)
    n1 = vmap.add(pts, rgb, inten)
    assert n1 == len(vmap) == voxel_downsample(pts, 0.1).shape[0]
    assert vmap.add(pts, rgb, inten) == 0  # same points again: nothing new
    # attributes stay aligned with their points
    # (the map keeps the same first-seen points, in voxel-key order within one add)
    first = voxel_downsample(pts, 0.1)

    def rows(xyz: np.ndarray, c: np.ndarray, i: np.ndarray) -> list[tuple[float, ...]]:
        return sorted(map(tuple, np.column_stack([xyz, c, i]).tolist()))

    assert rows(vmap.xyz, vmap.rgb, vmap.intensity) == rows(pts[first], rgb[first], inten[first])
    n3 = vmap.add(pts + np.float32(5.0))
    assert n3 > 0 and len(vmap) == n1 + n3
    # one point per voxel overall
    keys = np.floor(vmap.xyz.astype(np.float64) / 0.1).astype(np.int64)
    assert np.unique(keys, axis=0).shape[0] == len(vmap)
    assert vmap.rgb.shape == (len(vmap), 3) and vmap.intensity.shape == (len(vmap),)


def test_voxel_map_stays_bounded() -> None:
    vmap = VoxelMap(voxel=0.1, max_points=2000)
    rng = np.random.default_rng(2)
    for i in range(40):
        vmap.add(rng.uniform(0, 3, size=(500, 3)).astype(np.float32) + np.float32(i * 3.0))
        assert len(vmap) <= 2000
    assert vmap.evicted > 0
    # the newest points survive, the oldest were evicted
    assert vmap.xyz[:, 0].min() > 30.0
    # evicted voxels can be re-observed
    assert vmap.add(np.array([[0.5, 0.5, 0.5]], dtype=np.float32)) == 1


def test_map_builder_places_clouds_in_world() -> None:
    builder = MapBuilder(voxel=0.05)
    cloud = p.decode_frame(p.encode_point_cloud(0, 0.0, "depth_cam", p.make_points([[2.0, 0.0, 0.0]], [[1, 2, 3]], [9])))
    assert builder.process(cloud) == 0 and builder.clouds_skipped_no_tf == 1
    for frame in (
        p.encode_transform(0, 0.0, "base_link", "map", (10, 5, -30), quat_from_euler(0, 0, math.pi / 2)),
        p.encode_transform(1, 0.0, "depth_cam", "base_link", (1, 0, 0), (0, 0, 0, 1)),
    ):
        builder.process(p.decode_frame(frame))
    assert builder.process(cloud) == 1
    assert np.allclose(builder.map.xyz, [[10.0, 8.0, -30.0]], atol=1e-5)
    assert builder.map.rgb.tolist() == [[1, 2, 3]] and builder.map.intensity.tolist() == [9]
