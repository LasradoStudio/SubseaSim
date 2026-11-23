"""SubseaSim wire protocol v1: encode, decode, stream resync and a reconnecting TCP client.

See ``Docs/PROTOCOL.md``. Everything is little-endian and in ROS conventions (REP-103):
metres, radians, right-handed, X forward, Y left, Z up.

Frame layout (40-byte header, then ``payload_size`` bytes)::

    0   char[4]  magic "SSIM"
    4   uint16   version (1)
    6   uint16   msg_type
    8   uint32   seq
    12  float64  stamp (simulation seconds)
    20  char[16] frame_id (ASCII, NUL padded)
    36  uint32   payload_size
"""

from __future__ import annotations

import json
import socket
import struct
import time
from dataclasses import dataclass
from enum import IntEnum
from pathlib import Path
from typing import Any, Iterator, Mapping, Union

import numpy as np
import numpy.typing as npt

MAGIC = b"SSIM"
VERSION = 1
DEFAULT_PORT = 9870
HEADER_SIZE = 40
FRAME_ID_LEN = 16
#: Upper bound on ``payload_size`` accepted by the stream parser. A header announcing more
#: than this is treated as corruption and triggers a resync (4 M points is 64 MiB).
MAX_PAYLOAD_SIZE = 64 * 1024 * 1024

FLAG_RGB = 0x1
FLAG_INTENSITY = 0x2

_HEADER = struct.Struct("<4sHHId16sI")
_CLOUD_PREFIX = struct.Struct("<II")
_TRANSFORM = struct.Struct("<16s13d")
_SONAR_PREFIX = struct.Struct("<HHffff")
TRANSFORM_PAYLOAD_SIZE = _TRANSFORM.size

assert _HEADER.size == HEADER_SIZE
assert TRANSFORM_PAYLOAD_SIZE == 120

#: The 16-byte point record. ``np.frombuffer`` with this dtype decodes a payload zero-copy.
POINT_DTYPE = np.dtype(
    [
        ("x", "<f4"),
        ("y", "<f4"),
        ("z", "<f4"),
        ("r", "u1"),
        ("g", "u1"),
        ("b", "u1"),
        ("intensity", "u1"),
    ]
)
assert POINT_DTYPE.itemsize == 16

PointArray = npt.NDArray[np.void]


class MsgType(IntEnum):
    POINT_CLOUD = 1
    TRANSFORM = 2
    TELEMETRY = 3
    COMMAND = 4
    SONAR_IMAGE = 5


_KNOWN_TYPES = frozenset(int(t) for t in MsgType)


class ProtocolError(ValueError):
    """A frame was structurally valid enough to be framed but its contents are malformed."""


# --------------------------------------------------------------------------------------
# Messages
# --------------------------------------------------------------------------------------


@dataclass(frozen=True, slots=True)
class Header:
    msg_type: int
    seq: int
    stamp: float
    frame_id: str
    payload_size: int
    version: int = VERSION


@dataclass(frozen=True, slots=True)
class RawFrame:
    """A framed but undecoded message. Also what :func:`decode` returns for unknown types."""

    header: Header
    payload: bytes

    def to_bytes(self) -> bytes:
        h = self.header
        return encode_frame(h.msg_type, h.seq, h.stamp, h.frame_id, self.payload, version=h.version)


@dataclass(frozen=True, slots=True, eq=False)
class PointCloud:
    """Points in the sensor frame ``header.frame_id``.

    ``points`` is a read-only structured array (:data:`POINT_DTYPE`) that aliases the
    received payload; the accessors below are zero-copy views into it.
    """

    header: Header
    flags: int
    points: PointArray

    @property
    def num_points(self) -> int:
        return int(self.points.shape[0])

    @property
    def has_rgb(self) -> bool:
        return bool(self.flags & FLAG_RGB)

    @property
    def has_intensity(self) -> bool:
        return bool(self.flags & FLAG_INTENSITY)

    @property
    def xyz(self) -> npt.NDArray[np.float32]:
        """(N, 3) float32 view."""
        return points_xyz(self.points)

    @property
    def rgb(self) -> npt.NDArray[np.uint8]:
        """(N, 3) uint8 view. Only meaningful if :attr:`has_rgb`."""
        return points_rgb(self.points)

    @property
    def intensity(self) -> npt.NDArray[np.uint8]:
        """(N,) uint8 view. Only meaningful if :attr:`has_intensity`."""
        return points_intensity(self.points)


@dataclass(frozen=True, slots=True, eq=False)
class Transform:
    """Pose of ``header.frame_id`` (child) expressed in ``parent_frame``, plus child-frame twist."""

    header: Header
    parent_frame: str
    position: npt.NDArray[np.float64]  # (3,) x y z, metres
    orientation: npt.NDArray[np.float64]  # (4,) quaternion x y z w
    linear_vel: npt.NDArray[np.float64]  # (3,) m/s in the child frame
    angular_vel: npt.NDArray[np.float64]  # (3,) rad/s in the child frame

    @property
    def child_frame(self) -> str:
        return self.header.frame_id


@dataclass(frozen=True, slots=True, eq=False)
class SonarImage:
    """One imaging-sonar ping, formed in the sonar's own frame `header.frame_id`.

    `intensity` is a read-only (num_bins, num_beams) uint8 view: rows are range bins from
    `min_range` outwards, columns are beams from port (left, positive azimuth in ROS) to starboard.
    """

    header: Header
    hfov: float  # radians, full horizontal field of view
    min_range: float  # metres
    max_range: float  # metres
    vertical_aperture: float  # radians
    intensity: npt.NDArray[np.uint8]

    @property
    def num_beams(self) -> int:
        return int(self.intensity.shape[1])

    @property
    def num_bins(self) -> int:
        return int(self.intensity.shape[0])

    @property
    def beam_angles(self) -> npt.NDArray[np.float64]:
        """Azimuth of each beam centre in ROS convention (positive = port/left), radians."""
        n = self.num_beams
        return self.hfov / 2.0 - (np.arange(n) + 0.5) * (self.hfov / n)

    @property
    def bin_ranges(self) -> npt.NDArray[np.float64]:
        n = self.num_bins
        return self.min_range + (np.arange(n) + 0.5) * ((self.max_range - self.min_range) / n)


@dataclass(frozen=True, slots=True)
class Telemetry:
    header: Header
    data: dict[str, Any]


@dataclass(frozen=True, slots=True)
class Command:
    header: Header
    data: dict[str, Any]


Message = Union[PointCloud, Transform, Telemetry, Command, SonarImage, RawFrame]


# --------------------------------------------------------------------------------------
# Point array helpers
# --------------------------------------------------------------------------------------


def points_xyz(points: PointArray) -> npt.NDArray[np.float32]:
    """Zero-copy (N, 3) float32 view of a contiguous :data:`POINT_DTYPE` array."""
    return np.ascontiguousarray(points).view("<f4").reshape(-1, 4)[:, :3]


def points_rgb(points: PointArray) -> npt.NDArray[np.uint8]:
    """Zero-copy (N, 3) uint8 view of the r, g, b bytes."""
    return np.ascontiguousarray(points).view(np.uint8).reshape(-1, 16)[:, 12:15]


def points_intensity(points: PointArray) -> npt.NDArray[np.uint8]:
    """Zero-copy (N,) uint8 view of the intensity byte."""
    return np.ascontiguousarray(points).view(np.uint8).reshape(-1, 16)[:, 15]


def make_points(
    xyz: npt.ArrayLike,
    rgb: npt.ArrayLike | None = None,
    intensity: npt.ArrayLike | None = None,
) -> PointArray:
    """Build a :data:`POINT_DTYPE` array from an (N, 3) xyz array and optional attributes."""
    xyz_arr = np.asarray(xyz, dtype=np.float32).reshape(-1, 3)
    pts = np.zeros(xyz_arr.shape[0], dtype=POINT_DTYPE)
    raw_f = pts.view("<f4").reshape(-1, 4)
    raw_f[:, :3] = xyz_arr
    raw_b = pts.view(np.uint8).reshape(-1, 16)
    if rgb is not None:
        raw_b[:, 12:15] = np.asarray(rgb, dtype=np.uint8).reshape(-1, 3)
    if intensity is not None:
        raw_b[:, 15] = np.asarray(intensity, dtype=np.uint8).reshape(-1)
    return pts


# --------------------------------------------------------------------------------------
# Encoding
# --------------------------------------------------------------------------------------


def _encode_name(name: str, what: str) -> bytes:
    try:
        raw = name.encode("ascii")
    except UnicodeEncodeError as exc:
        raise ProtocolError(f"{what} must be ASCII: {name!r}") from exc
    if len(raw) > FRAME_ID_LEN:
        raise ProtocolError(f"{what} longer than {FRAME_ID_LEN} bytes: {name!r}")
    return raw  # struct pads with NULs


def _decode_name(raw: bytes) -> str:
    return raw.split(b"\x00", 1)[0].decode("ascii", errors="replace")


def encode_header(
    msg_type: int, seq: int, stamp: float, frame_id: str, payload_size: int, *, version: int = VERSION
) -> bytes:
    return _HEADER.pack(
        MAGIC,
        version,
        int(msg_type),
        seq & 0xFFFFFFFF,
        float(stamp),
        _encode_name(frame_id, "frame_id"),
        payload_size,
    )


def encode_frame(
    msg_type: int,
    seq: int,
    stamp: float,
    frame_id: str,
    payload: bytes | bytearray | memoryview,
    *,
    version: int = VERSION,
) -> bytes:
    return encode_header(msg_type, seq, stamp, frame_id, len(payload), version=version) + bytes(payload)


def encode_point_cloud(
    seq: int,
    stamp: float,
    frame_id: str,
    points: PointArray,
    flags: int = FLAG_RGB | FLAG_INTENSITY,
) -> bytes:
    if points.dtype != POINT_DTYPE:
        raise ProtocolError(f"points must have dtype POINT_DTYPE, got {points.dtype}")
    n = int(points.shape[0])
    body = np.ascontiguousarray(points).tobytes()
    header = encode_header(MsgType.POINT_CLOUD, seq, stamp, frame_id, _CLOUD_PREFIX.size + len(body))
    return b"".join((header, _CLOUD_PREFIX.pack(n, flags), body))


def encode_transform(
    seq: int,
    stamp: float,
    frame_id: str,
    parent_frame: str,
    position: npt.ArrayLike,
    orientation: npt.ArrayLike,
    linear_vel: npt.ArrayLike = (0.0, 0.0, 0.0),
    angular_vel: npt.ArrayLike = (0.0, 0.0, 0.0),
) -> bytes:
    """``frame_id`` is the child; ``orientation`` is a quaternion (x, y, z, w)."""
    p = np.asarray(position, dtype=np.float64).reshape(3)
    q = np.asarray(orientation, dtype=np.float64).reshape(4)
    v = np.asarray(linear_vel, dtype=np.float64).reshape(3)
    w = np.asarray(angular_vel, dtype=np.float64).reshape(3)
    payload = _TRANSFORM.pack(_encode_name(parent_frame, "parent_frame"), *p, *q, *v, *w)
    return encode_frame(MsgType.TRANSFORM, seq, stamp, frame_id, payload)


def _encode_json(data: Mapping[str, Any]) -> bytes:
    return json.dumps(dict(data), separators=(",", ":"), allow_nan=False).encode("utf-8")


def encode_telemetry(seq: int, stamp: float, data: Mapping[str, Any], frame_id: str = "base_link") -> bytes:
    return encode_frame(MsgType.TELEMETRY, seq, stamp, frame_id, _encode_json(data))


def encode_sonar_image(
    seq: int,
    stamp: float,
    intensity: npt.ArrayLike,
    hfov: float,
    min_range: float,
    max_range: float,
    vertical_aperture: float = 0.35,
    frame_id: str = "sonar",
) -> bytes:
    """Encode a (num_bins, num_beams) uint8 sonar image."""
    img = np.ascontiguousarray(intensity, dtype=np.uint8)
    if img.ndim != 2:
        raise ValueError("sonar intensity must be 2-D (num_bins, num_beams)")
    bins, beams = img.shape
    if not (0 < beams <= 0xFFFF and 0 < bins <= 0xFFFF):
        raise ValueError("sonar image dimensions out of range")
    prefix = _SONAR_PREFIX.pack(beams, bins, hfov, min_range, max_range, vertical_aperture)
    return encode_frame(MsgType.SONAR_IMAGE, seq, stamp, frame_id, prefix + img.tobytes())


def encode_command(seq: int, stamp: float, data: Mapping[str, Any], frame_id: str = "") -> bytes:
    return encode_frame(MsgType.COMMAND, seq, stamp, frame_id, _encode_json(data))


# --------------------------------------------------------------------------------------
# Decoding
# --------------------------------------------------------------------------------------


def decode_header(buf: bytes | bytearray | memoryview, offset: int = 0) -> Header:
    """Decode a 40-byte header. Raises :class:`ProtocolError` on bad magic or version."""
    if len(buf) - offset < HEADER_SIZE:
        raise ProtocolError("buffer shorter than a header")
    magic, version, msg_type, seq, stamp, frame_id, payload_size = _HEADER.unpack_from(buf, offset)
    if magic != MAGIC:
        raise ProtocolError(f"bad magic {magic!r}")
    if version != VERSION:
        raise ProtocolError(f"unsupported protocol version {version}")
    return Header(msg_type, seq, stamp, _decode_name(frame_id), payload_size, version)


def _decode_json(payload: bytes) -> dict[str, Any]:
    try:
        obj = json.loads(payload.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ProtocolError(f"invalid JSON payload: {exc}") from exc
    if not isinstance(obj, dict):
        raise ProtocolError("JSON payload is not an object")
    return obj


def decode(frame: RawFrame) -> Message:
    """Decode a framed message. Unknown ``msg_type`` values are returned unchanged."""
    header, payload = frame.header, frame.payload
    if len(payload) != header.payload_size:
        raise ProtocolError("payload length does not match header.payload_size")
    mt = header.msg_type
    if mt == MsgType.POINT_CLOUD:
        if len(payload) < _CLOUD_PREFIX.size:
            raise ProtocolError("point cloud payload shorter than its 8-byte prefix")
        num_points, flags = _CLOUD_PREFIX.unpack_from(payload, 0)
        if _CLOUD_PREFIX.size + num_points * POINT_DTYPE.itemsize != len(payload):
            raise ProtocolError(
                f"point cloud payload is {len(payload)} bytes but num_points={num_points}"
            )
        points = np.frombuffer(payload, dtype=POINT_DTYPE, count=num_points, offset=_CLOUD_PREFIX.size)
        return PointCloud(header, flags, points)
    if mt == MsgType.TRANSFORM:
        if len(payload) != TRANSFORM_PAYLOAD_SIZE:
            raise ProtocolError(f"transform payload must be 120 bytes, got {len(payload)}")
        parent = _decode_name(payload[:FRAME_ID_LEN])
        vals = np.frombuffer(payload, dtype="<f8", count=13, offset=FRAME_ID_LEN)
        return Transform(header, parent, vals[0:3], vals[3:7], vals[7:10], vals[10:13])
    if mt == MsgType.TELEMETRY:
        return Telemetry(header, _decode_json(payload))
    if mt == MsgType.COMMAND:
        return Command(header, _decode_json(payload))
    if mt == MsgType.SONAR_IMAGE:
        if len(payload) < _SONAR_PREFIX.size:
            raise ProtocolError("sonar payload shorter than its 20-byte prefix")
        beams, bins, hfov, rmin, rmax, aperture = _SONAR_PREFIX.unpack_from(payload, 0)
        if _SONAR_PREFIX.size + beams * bins != len(payload):
            raise ProtocolError(f"sonar payload is {len(payload)} bytes but image is {bins}x{beams}")
        img = np.frombuffer(payload, dtype=np.uint8, count=beams * bins, offset=_SONAR_PREFIX.size).reshape(bins, beams)
        return SonarImage(header, hfov, rmin, rmax, aperture, img)
    return frame


def decode_frame(buf: bytes | bytearray | memoryview) -> Message:
    """Decode exactly one complete frame from ``buf``."""
    header = decode_header(buf)
    end = HEADER_SIZE + header.payload_size
    if len(buf) != end:
        raise ProtocolError(f"expected a {end}-byte frame, got {len(buf)} bytes")
    return decode(RawFrame(header, bytes(buf[HEADER_SIZE:end])))


# --------------------------------------------------------------------------------------
# Stream parsing (sans-IO)
# --------------------------------------------------------------------------------------


class StreamParser:
    """Incremental frame splitter that resynchronises on the ``SSIM`` magic.

    Feed it arbitrary chunks of the byte stream; it returns every complete frame found.
    Anything that is not a plausible frame (garbage, a truncated frame followed by a new
    one, a header with the wrong version or an absurd size) is skipped byte-by-byte until
    the next valid header. The protocol has no checksum, so a header is accepted when:
    magic matches, version is 1, ``payload_size`` is within ``max_payload``, and for the
    fixed-layout types the size is self-consistent (transform == 120 bytes, point cloud
    == 8 + 16 * num_points).
    """

    def __init__(self, max_payload: int = MAX_PAYLOAD_SIZE) -> None:
        self._buf = bytearray()
        self._max_payload = max_payload
        self._dirty = False
        self.frames = 0
        self.bytes_discarded = 0
        self.resyncs = 0
        self.truncated_frames = 0

    def reset(self) -> None:
        """Drop buffered bytes (call after a reconnect). Counters are kept."""
        self._buf.clear()
        self._dirty = False

    @property
    def buffered(self) -> int:
        return len(self._buf)

    def _plausible(self, pos: int, n: int) -> tuple[Header | None, bool]:
        """Return (header, need_more). header is None if the candidate must be skipped."""
        buf = self._buf
        _, version, msg_type, seq, stamp, frame_id, size = _HEADER.unpack_from(buf, pos)
        if version != VERSION or size > self._max_payload:
            return None, False
        if msg_type == MsgType.TRANSFORM and size != TRANSFORM_PAYLOAD_SIZE:
            return None, False
        if msg_type == MsgType.POINT_CLOUD:
            if size < _CLOUD_PREFIX.size or (size - _CLOUD_PREFIX.size) % POINT_DTYPE.itemsize:
                return None, False
            if n - pos < HEADER_SIZE + _CLOUD_PREFIX.size:
                return None, True
            (num_points,) = struct.unpack_from("<I", buf, pos + HEADER_SIZE)
            if _CLOUD_PREFIX.size + num_points * POINT_DTYPE.itemsize != size:
                return None, False
        return Header(msg_type, seq, stamp, _decode_name(frame_id), size, version), False

    def _embedded_header(self, start: int, end: int, n: int) -> int:
        """Offset of a plausible known-type header inside buf[start:end], or -1.

        A complete-looking frame whose payload contains another frame header is really a
        truncated frame followed by intact ones (e.g. the sender dropped the tail of a
        frame for a slow client). Real point data matching magic + version + a consistent
        size by chance is vanishingly unlikely, and JSON payloads cannot contain it.
        """
        buf = self._buf
        j = buf.find(MAGIC, start, end)
        while j >= 0:
            if n - j >= HEADER_SIZE:
                header, need_more = self._plausible(j, n)
                if header is not None and not need_more and header.msg_type in _KNOWN_TYPES:
                    return j
            j = buf.find(MAGIC, j + 1, end)
        return -1

    def feed(self, data: bytes | bytearray | memoryview) -> list[RawFrame]:
        buf = self._buf
        buf += data
        out: list[RawFrame] = []
        pos = 0
        n = len(buf)
        while True:
            idx = buf.find(MAGIC, pos)
            if idx < 0:
                # No magic: discard everything except a possible split-magic tail.
                keep_from = max(pos, n - (len(MAGIC) - 1))
                if keep_from > pos:
                    self.bytes_discarded += keep_from - pos
                    self._dirty = True
                pos = keep_from
                break
            if idx > pos:
                self.bytes_discarded += idx - pos
                self._dirty = True
                pos = idx
            if n - pos < HEADER_SIZE:
                break
            header, need_more = self._plausible(pos, n)
            if need_more:
                break
            if header is None:
                pos += 1
                self.bytes_discarded += 1
                self._dirty = True
                continue
            end = pos + HEADER_SIZE + header.payload_size
            if n < end:
                break
            inner = self._embedded_header(pos + len(MAGIC), end, n)
            if inner >= 0:
                # This frame was cut short by the sender: drop it, resume at the real one.
                self.bytes_discarded += inner - pos
                self.truncated_frames += 1
                self._dirty = True
                pos = inner
                continue
            with memoryview(buf) as mv:
                payload = bytes(mv[pos + HEADER_SIZE : end])
            out.append(RawFrame(header, payload))
            self.frames += 1
            if self._dirty:
                self.resyncs += 1
                self._dirty = False
            pos = end
        if pos:
            del buf[:pos]
        return out


def iter_frames(data: bytes | bytearray | memoryview, chunk_size: int = 1 << 20) -> Iterator[RawFrame]:
    """Split an in-memory byte string into frames (with resync)."""
    parser = StreamParser()
    view = memoryview(data)
    for off in range(0, len(view), chunk_size):
        yield from parser.feed(view[off : off + chunk_size])


def read_recording(path: str | Path, chunk_size: int = 1 << 20) -> Iterator[RawFrame]:
    """Iterate the frames of a raw stream capture (see :mod:`subsea_bridge.recorder`)."""
    parser = StreamParser()
    with open(path, "rb") as fh:
        while True:
            chunk = fh.read(chunk_size)
            if not chunk:
                break
            yield from parser.feed(chunk)


# --------------------------------------------------------------------------------------
# TCP client
# --------------------------------------------------------------------------------------


class StreamClient:
    """TCP client for the SubseaSim stream with resync and automatic reconnect.

    ``poll()`` never blocks for much longer than its timeout and returns whatever complete
    messages arrived; ``messages()`` is a blocking generator on top of it. With
    ``reconnect=True`` (default) connection failures are retried forever with exponential
    backoff; with ``reconnect=False`` they raise :class:`ConnectionError`.
    """

    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = DEFAULT_PORT,
        *,
        reconnect: bool = True,
        connect_timeout: float = 3.0,
        reconnect_delay: float = 0.25,
        max_reconnect_delay: float = 4.0,
        recv_size: int = 1 << 20,
    ) -> None:
        self.host = host
        self.port = port
        self.reconnect = reconnect
        self.connect_timeout = connect_timeout
        self._base_delay = reconnect_delay
        self._max_delay = max_reconnect_delay
        self._delay = reconnect_delay
        self._next_attempt = 0.0
        self._sock: socket.socket | None = None
        self._closed = False
        self._recv_buf = bytearray(recv_size)
        self._recv_view = memoryview(self._recv_buf)
        self._cmd_seq = 0
        self.parser = StreamParser()
        self.connects = 0
        self.bytes_received = 0
        self.decode_errors = 0
        self.last_error: str | None = None

    # -- lifecycle ---------------------------------------------------------------

    def __enter__(self) -> "StreamClient":
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()

    @property
    def connected(self) -> bool:
        return self._sock is not None

    @property
    def closed(self) -> bool:
        return self._closed

    def close(self) -> None:
        """Stop the client. Safe to call from another thread to unblock ``messages()``."""
        self._closed = True
        self._drop()

    def _drop(self) -> None:
        sock, self._sock = self._sock, None
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass

    def _fail(self, exc: BaseException) -> None:
        self.last_error = f"{type(exc).__name__}: {exc}"
        self._drop()
        self.parser.reset()
        if not self.reconnect and not self._closed:
            raise ConnectionError(f"SubseaSim stream {self.host}:{self.port}: {self.last_error}") from exc
        self._next_attempt = time.monotonic() + self._delay
        self._delay = min(self._delay * 2.0, self._max_delay)

    def _ensure_connected(self, timeout: float) -> bool:
        if self._sock is not None:
            return True
        wait = self._next_attempt - time.monotonic()
        if wait > 0:
            time.sleep(min(wait, max(timeout, 0.0)))
            if time.monotonic() < self._next_attempt:
                return False
        if self._closed:
            return False
        try:
            sock = socket.create_connection((self.host, self.port), timeout=self.connect_timeout)
        except OSError as exc:
            self._fail(exc)
            return False
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 << 20)
        except OSError:
            pass
        self._sock = sock
        self._delay = self._base_delay
        self.connects += 1
        self.parser.reset()
        return True

    # -- reading -----------------------------------------------------------------

    def poll_raw(self, timeout: float = 0.5) -> list[RawFrame]:
        """Wait up to ``timeout`` seconds for data and return the complete frames received."""
        if self._closed or not self._ensure_connected(timeout):
            return []
        sock = self._sock
        assert sock is not None
        try:
            sock.settimeout(max(timeout, 1e-3))
            n = sock.recv_into(self._recv_view)
        except (socket.timeout, BlockingIOError):
            return []
        except OSError as exc:
            if not self._closed:
                self._fail(exc)
            return []
        if n == 0:
            if not self._closed:
                self._fail(ConnectionResetError("connection closed by the sim"))
            return []
        self.bytes_received += n
        return self.parser.feed(self._recv_view[:n])

    def poll(self, timeout: float = 0.5) -> list[Message]:
        """Like :meth:`poll_raw` but decoded. Malformed frames are counted and skipped."""
        out: list[Message] = []
        for frame in self.poll_raw(timeout):
            try:
                out.append(decode(frame))
            except ProtocolError as exc:
                self.decode_errors += 1
                self.last_error = f"ProtocolError: {exc}"
        return out

    def messages(self, poll_timeout: float = 0.5) -> Iterator[Message]:
        """Yield decoded messages until :meth:`close` is called."""
        while not self._closed:
            yield from self.poll(poll_timeout)

    # -- writing -----------------------------------------------------------------

    def send_command(self, data: Mapping[str, Any]) -> bool:
        """Send an optional msg_type 4 command frame. Returns False if not connected."""
        sock = self._sock
        if sock is None:
            return False
        frame = encode_command(self._cmd_seq, 0.0, data)
        self._cmd_seq += 1
        try:
            sock.settimeout(2.0)
            sock.sendall(frame)
        except OSError as exc:
            if not self._closed:
                self._fail(exc)
            return False
        return True
