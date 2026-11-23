"""Rigid-transform helpers (REP-103: X forward, Y left, Z up; quaternions are x, y, z, w)."""

from __future__ import annotations

import math

import numpy as np
import numpy.typing as npt

from .protocol import Transform

Mat4 = npt.NDArray[np.float64]


def quat_from_euler(roll: float, pitch: float, yaw: float) -> npt.NDArray[np.float64]:
    """Quaternion (x, y, z, w) from fixed-axis roll/pitch/yaw (ROS convention, R = Rz Ry Rx)."""
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return np.array(
        [
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        ],
        dtype=np.float64,
    )


def quat_to_matrix(q: npt.ArrayLike) -> npt.NDArray[np.float64]:
    """3x3 rotation matrix from a quaternion (x, y, z, w). The input is normalised."""
    x, y, z, w = (float(v) for v in np.asarray(q, dtype=np.float64).reshape(4))
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def yaw_from_quat(q: npt.ArrayLike) -> float:
    x, y, z, w = (float(v) for v in np.asarray(q, dtype=np.float64).reshape(4))
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def pose_matrix(position: npt.ArrayLike, orientation: npt.ArrayLike) -> Mat4:
    """4x4 homogeneous matrix mapping child-frame points into the parent frame."""
    T = np.eye(4)
    T[:3, :3] = quat_to_matrix(orientation)
    T[:3, 3] = np.asarray(position, dtype=np.float64).reshape(3)
    return T


def transform_points(T: Mat4, xyz: npt.ArrayLike) -> npt.NDArray[np.float32]:
    """Apply a 4x4 transform to an (N, 3) array; returns float32."""
    pts = np.asarray(xyz, dtype=np.float32).reshape(-1, 3)
    R = T[:3, :3].astype(np.float32)
    t = T[:3, 3].astype(np.float32)
    return pts @ R.T + t


class TransformTree:
    """Latest-value transform tree (no time interpolation), keyed by child frame."""

    def __init__(self) -> None:
        self._edges: dict[str, tuple[str, Mat4]] = {}

    def set(self, parent: str, child: str, T_parent_child: Mat4) -> None:
        self._edges[child] = (parent, T_parent_child)

    def update(self, msg: Transform) -> None:
        self.set(msg.parent_frame, msg.child_frame, pose_matrix(msg.position, msg.orientation))

    def parent_of(self, frame: str) -> str | None:
        edge = self._edges.get(frame)
        return edge[0] if edge else None

    def chain(self, frame: str) -> list[str]:
        """Frames from the root down to ``frame`` (inclusive)."""
        out = [frame]
        while (parent := self.parent_of(out[-1])) is not None and len(out) <= len(self._edges):
            out.append(parent)
        return out[::-1]

    def lookup(self, frame: str, target: str = "map") -> Mat4 | None:
        """Transform mapping points in ``frame`` into ``target`` (an ancestor), or None."""
        T = np.eye(4)
        cur = frame
        for _ in range(len(self._edges) + 1):
            if cur == target:
                return T
            edge = self._edges.get(cur)
            if edge is None:
                return None
            cur, T = edge[0], edge[1] @ T
        return None
