"""Sonar message round trip and scan conversion."""

import numpy as np
import pytest

from subsea_bridge.protocol import MsgType, ProtocolError, SonarImage, StreamParser, decode, decode_frame, encode_sonar_image
from subsea_bridge.sonar import copper, render_fan


def make_image(bins: int = 64, beams: int = 32) -> np.ndarray:
    img = np.zeros((bins, beams), dtype=np.uint8)
    img[bins // 2, :] = 255  # an arc of strong returns at mid range
    img[:, 0] = 40  # faint port-most beam
    return img


def test_round_trip() -> None:
    img = make_image()
    frame = encode_sonar_image(3, 1.25, img, np.deg2rad(120), 0.5, 30.0, np.deg2rad(20))
    msg = decode_frame(frame)
    assert isinstance(msg, SonarImage)
    assert msg.header.msg_type == MsgType.SONAR_IMAGE and msg.header.seq == 3 and msg.header.frame_id == "sonar"
    assert (msg.num_bins, msg.num_beams) == img.shape
    assert np.array_equal(msg.intensity, img)
    assert msg.max_range == pytest.approx(30.0) and msg.hfov == pytest.approx(np.deg2rad(120), rel=1e-6)


def test_beam_angles_port_first() -> None:
    msg = decode_frame(encode_sonar_image(0, 0.0, make_image(beams=4), np.pi / 2, 0.5, 10.0))
    angles = msg.beam_angles
    assert angles[0] > 0 > angles[-1]  # ROS: positive azimuth is port
    assert np.allclose(angles, -angles[::-1])
    assert msg.bin_ranges[0] > 0.5 and msg.bin_ranges[-1] < 10.0


def test_size_mismatch_rejected() -> None:
    frame = bytearray(encode_sonar_image(0, 0.0, make_image(), 2.0, 0.5, 30.0))
    with pytest.raises(ProtocolError):
        # Truncate the image but keep the header's payload size consistent with what is left.
        raw = StreamParser().feed(bytes(frame))[0]
        decode(type(raw)(type(raw.header)(*[*vars_of(raw.header)[:4], raw.header.payload_size - 10]), raw.payload[:-10]))


def vars_of(header):  # helper: header fields in declaration order
    return [header.msg_type, header.seq, header.stamp, header.frame_id, header.payload_size]


def test_stream_parser_frames_sonar() -> None:
    frames = encode_sonar_image(0, 0.0, make_image(), 2.0, 0.5, 30.0) + encode_sonar_image(1, 0.1, make_image(), 2.0, 0.5, 30.0)
    raws = StreamParser().feed(frames)
    assert [decode(r).header.seq for r in raws] == [0, 1]


def test_render_fan_geometry() -> None:
    msg = decode_frame(encode_sonar_image(0, 0.0, make_image(), np.deg2rad(120), 0.5, 30.0))
    fan = render_fan(msg, width=400)
    assert fan.shape == (250, 400, 3) and fan.dtype == np.uint8
    # Outside the fan (top corners) stays background; the mid-range arc is bright on the centre line.
    assert fan[2, 2].sum() == 0 and fan[2, -3].sum() == 0
    column = fan[:, 200].astype(int).sum(axis=1)
    assert column.max() > 400


def test_copper_monotonic() -> None:
    ramp = copper(np.linspace(0, 1, 32)).astype(int).sum(axis=1)
    assert np.all(np.diff(ramp) >= 0) and ramp[0] < 30 and ramp[-1] > 550