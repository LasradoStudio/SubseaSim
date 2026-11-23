"""Voxel downsampling and a bounded, accumulated world-frame map (numpy only)."""

from __future__ import annotations

import numpy as np
import numpy.typing as npt

from .geometry import TransformTree, transform_points
from .protocol import Message, PointCloud, Transform

_OFFSET = 1 << 20  # voxel indices must lie in [-2**20, 2**20) per axis
_BITS = 21


def voxel_keys(xyz: npt.NDArray[np.float32], voxel: float) -> npt.NDArray[np.int64]:
    """Pack integer voxel coordinates into one int64 key per point."""
    if voxel <= 0:
        raise ValueError("voxel size must be positive")
    ijk = np.floor(xyz.astype(np.float64) * (1.0 / voxel)).astype(np.int64) + _OFFSET
    np.clip(ijk, 0, (1 << _BITS) - 1, out=ijk)
    return (ijk[:, 0] << (2 * _BITS)) | (ijk[:, 1] << _BITS) | ijk[:, 2]


def voxel_downsample(xyz: npt.ArrayLike, voxel: float) -> npt.NDArray[np.intp]:
    """Indices (ascending) of the first finite point in each occupied voxel.

    Use the result to index the positions and any per-point attributes.
    """
    pts = np.asarray(xyz, dtype=np.float32).reshape(-1, 3)
    finite = np.flatnonzero(np.isfinite(pts).all(axis=1))
    if finite.size == 0:
        return finite
    _, first = np.unique(voxel_keys(pts[finite], voxel), return_index=True)
    first.sort()
    return finite[first]


class VoxelMap:
    """Accumulated point map holding at most one point per voxel and at most ``max_points``.

    When the cap is exceeded the oldest ~10 % of points are evicted, so memory and the
    cost of re-logging the map stay bounded on long runs.
    """

    def __init__(self, voxel: float = 0.10, max_points: int = 400_000) -> None:
        if voxel <= 0:
            raise ValueError("voxel size must be positive")
        if max_points <= 0:
            raise ValueError("max_points must be positive")
        self.voxel = float(voxel)
        self.max_points = int(max_points)
        self._sorted_keys = np.empty(0, dtype=np.int64)
        self._keys = np.empty(0, dtype=np.int64)  # insertion order
        self._xyz = np.empty((0, 3), dtype=np.float32)
        self._rgb = np.empty((0, 3), dtype=np.uint8)
        self._intensity = np.empty(0, dtype=np.uint8)
        self.evicted = 0

    def __len__(self) -> int:
        return int(self._keys.shape[0])

    @property
    def xyz(self) -> npt.NDArray[np.float32]:
        return self._xyz

    @property
    def rgb(self) -> npt.NDArray[np.uint8]:
        return self._rgb

    @property
    def intensity(self) -> npt.NDArray[np.uint8]:
        return self._intensity

    def add(
        self,
        xyz: npt.ArrayLike,
        rgb: npt.ArrayLike | None = None,
        intensity: npt.ArrayLike | None = None,
    ) -> int:
        """Insert points whose voxel is not occupied yet. Returns the number added."""
        pts = np.asarray(xyz, dtype=np.float32).reshape(-1, 3)
        finite = np.flatnonzero(np.isfinite(pts).all(axis=1))
        if finite.size == 0:
            return 0
        keys, first = np.unique(voxel_keys(pts[finite], self.voxel), return_index=True)
        sk = self._sorted_keys
        pos = np.searchsorted(sk, keys)
        if sk.size:
            fresh = sk[np.minimum(pos, sk.size - 1)] != keys
        else:
            fresh = np.ones(keys.shape, dtype=bool)
        if not fresh.any():
            return 0
        new_keys = keys[fresh]
        src = finite[first[fresh]]
        n_new = int(new_keys.shape[0])

        self._sorted_keys = np.insert(sk, pos[fresh], new_keys)
        self._keys = np.concatenate((self._keys, new_keys))
        self._xyz = np.concatenate((self._xyz, pts[src]))
        if rgb is not None:
            new_rgb = np.asarray(rgb, dtype=np.uint8).reshape(-1, 3)[src]
        else:
            new_rgb = np.full((n_new, 3), 200, dtype=np.uint8)
        self._rgb = np.concatenate((self._rgb, new_rgb))
        if intensity is not None:
            new_int = np.asarray(intensity, dtype=np.uint8).reshape(-1)[src]
        else:
            new_int = np.zeros(n_new, dtype=np.uint8)
        self._intensity = np.concatenate((self._intensity, new_int))

        if len(self) > self.max_points:
            self._evict(len(self) - self.max_points + self.max_points // 10)
        return n_new

    def _evict(self, count: int) -> None:
        count = min(count, len(self))
        gone = self._keys[:count]
        self._sorted_keys = np.setdiff1d(self._sorted_keys, gone, assume_unique=True)
        self._keys = self._keys[count:]
        self._xyz = self._xyz[count:]
        self._rgb = self._rgb[count:]
        self._intensity = self._intensity[count:]
        self.evicted += count


class MapBuilder:
    """Feeds decoded stream messages into a :class:`TransformTree` and a :class:`VoxelMap`.

    Clouds are placed with the most recent transforms received before them (the sim sends
    the transforms for a tick ahead of that tick's cloud). Clouds that arrive before the
    chain to ``world_frame`` is known are skipped.
    """

    def __init__(
        self,
        voxel: float = 0.10,
        max_points: int = 400_000,
        world_frame: str = "map",
        every: int = 1,
    ) -> None:
        self.tf = TransformTree()
        self.map = VoxelMap(voxel, max_points)
        self.world_frame = world_frame
        self.every = max(1, every)
        self.clouds_seen = 0
        self.clouds_integrated = 0
        self.clouds_skipped_no_tf = 0

    def process(self, msg: Message) -> int:
        """Returns the number of map points added by this message."""
        if isinstance(msg, Transform):
            self.tf.update(msg)
            return 0
        if not isinstance(msg, PointCloud):
            return 0
        self.clouds_seen += 1
        if (self.clouds_seen - 1) % self.every or msg.num_points == 0:
            return 0
        T = self.tf.lookup(msg.header.frame_id, self.world_frame)
        if T is None:
            self.clouds_skipped_no_tf += 1
            return 0
        self.clouds_integrated += 1
        world = transform_points(T, msg.xyz)
        return self.map.add(
            world,
            msg.rgb if msg.has_rgb else None,
            msg.intensity if msg.has_intensity else None,
        )
