"""SubseaSim bridge: Python client tools for the SubseaSim point-cloud stream.

Importing this package needs only numpy. Rerun and ROS 2 are imported solely by
``viewer_rerun`` and ``ros2_bridge`` respectively.
"""

from .protocol import (
    DEFAULT_PORT,
    FLAG_INTENSITY,
    FLAG_RGB,
    HEADER_SIZE,
    MAGIC,
    POINT_DTYPE,
    VERSION,
    Command,
    Header,
    Message,
    MsgType,
    PointCloud,
    ProtocolError,
    RawFrame,
    StreamClient,
    StreamParser,
    Telemetry,
    Transform,
    decode,
    decode_frame,
    encode_command,
    encode_frame,
    encode_point_cloud,
    encode_telemetry,
    encode_transform,
    make_points,
)

__version__ = "0.1.0"

__all__ = [
    "DEFAULT_PORT",
    "FLAG_INTENSITY",
    "FLAG_RGB",
    "HEADER_SIZE",
    "MAGIC",
    "POINT_DTYPE",
    "VERSION",
    "Command",
    "Header",
    "Message",
    "MsgType",
    "PointCloud",
    "ProtocolError",
    "RawFrame",
    "StreamClient",
    "StreamParser",
    "Telemetry",
    "Transform",
    "decode",
    "decode_frame",
    "encode_command",
    "encode_frame",
    "encode_point_cloud",
    "encode_telemetry",
    "encode_transform",
    "make_points",
    "__version__",
]
