"""Sonar display helpers: polar-to-fan scan conversion and the copper colour map.

Mirrors ``USubseaSonarComponent::RenderFan`` on the Unreal side so the viewer, the recorder and
offline renders all show the same picture the pilot sees.
"""

from __future__ import annotations

import numpy as np
import numpy.typing as npt

from .protocol import SonarImage


def copper(values: npt.NDArray[np.floating]) -> npt.NDArray[np.uint8]:
    """Map 0..1 to the amber/copper ramp typical of imaging-sonar displays. Returns (..., 3) uint8."""
    c = np.clip(values, 0.0, 1.0)
    r = np.power(c, 0.75) * 255.0
    g = np.power(c, 1.35) * 215.0
    b = np.power(c, 2.6) * 150.0 + 6.0 * (1.0 - c)
    return np.clip(np.stack([r, g, b], axis=-1), 0, 255).astype(np.uint8)


def fan_geometry(width: int, height: int, hfov: float, pad: float = 4.0) -> tuple[float, float, float]:
    """Apex x, apex y and radius (pixels) of the largest fan that fits."""
    half = hfov / 2.0
    radius = min(height - 2.0 * pad, (width * 0.5 - pad) / max(np.sin(half), 0.2))
    return width * 0.5, height - pad, radius


def render_fan(
    sonar: SonarImage,
    width: int = 960,
    height: int | None = None,
    background: tuple[int, int, int] = (0, 0, 0),
    rings_every_m: float = 5.0,
) -> npt.NDArray[np.uint8]:
    """Scan-convert a polar sonar image into an (H, W, 3) fan image, apex at the bottom centre.

    Port (left) beams are drawn on the left, as a pilot would see them.
    """
    if height is None:
        height = int(round(width * 5 / 8))
    img = sonar.intensity.astype(np.float32) / 255.0
    bins, beams = img.shape
    half = sonar.hfov / 2.0
    cx, cy, radius = fan_geometry(width, height, sonar.hfov)

    ys, xs = np.mgrid[0:height, 0:width].astype(np.float32)
    dx = xs + 0.5 - cx
    dy = cy - (ys + 0.5)
    dist = np.hypot(dx, dy)
    angle = np.arctan2(dx, dy)  # 0 straight ahead, positive to the right (starboard)
    rng = dist / radius * sonar.max_range
    inside = (np.abs(angle) <= half) & (dist <= radius)

    # Beam 0 is port (left), i.e. most negative screen angle.
    beam_pos = (angle + half) / sonar.hfov * beams - 0.5
    bin_pos = (rng - sonar.min_range) / max(sonar.max_range - sonar.min_range, 1e-6) * bins - 0.5
    b0 = np.clip(np.floor(beam_pos).astype(np.intp), 0, beams - 1)
    r0 = np.clip(np.floor(bin_pos).astype(np.intp), 0, bins - 1)
    b1 = np.minimum(b0 + 1, beams - 1)
    r1 = np.minimum(r0 + 1, bins - 1)
    fb = np.clip(beam_pos - b0, 0.0, 1.0)
    fr = np.clip(bin_pos - r0, 0.0, 1.0)
    value = (img[r0, b0] * (1 - fb) + img[r0, b1] * fb) * (1 - fr) + (img[r1, b0] * (1 - fb) + img[r1, b1] * fb) * fr

    out = np.empty((height, width, 3), dtype=np.uint8)
    out[:] = background
    out[inside] = copper(value[inside])
    out[inside & (rng < sonar.min_range)] = (8, 10, 12)

    if rings_every_m > 0:
        ring_px = radius * rings_every_m / sonar.max_range
        ring = inside & (np.mod(dist, ring_px) < 1.0) & (dist > ring_px * 0.5)
        out[ring] = np.maximum(out[ring], np.array([70, 120, 120], dtype=np.uint8))
    edge = inside & (((half - np.abs(angle)) * dist < 1.2) | (radius - dist < 1.2))
    out[edge] = (120, 190, 190)
    return out
