"""Renders sensor data recorded from the stream (.ssim) into video frames, headlessly.

Modes
  build   time-lapse of the point-cloud map growing behind the vehicle (chase camera)
  orbit   slow orbit around the finished map
  sonar   sonar fan plus live depth-camera vs sonar plots, frame-synced to a game recording

Frames are written as PNGs (assemble with ffmpeg). Points are splatted with a depth sort in numpy,
so no GPU or 3D viewer is needed.

    python Scripts/render_sensor_video.py build Saved/Film/film.ssim Saved/Film/build --t0 60 --t1 160 --seconds 10
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFilter, ImageFont

from subsea_bridge.geometry import TransformTree, quat_to_matrix, transform_points
from subsea_bridge.protocol import PointCloud, SonarImage, Telemetry, Transform, decode, read_recording
from subsea_bridge.sonar import render_fan

FONT_PATH = "C:/Windows/Fonts/segoeui.ttf"
FONT_BOLD = "C:/Windows/Fonts/segoeuib.ttf"
BG_TOP = np.array([4, 12, 20], dtype=np.float32)
BG_BOTTOM = np.array([1, 4, 8], dtype=np.float32)
# Deep teal -> cyan -> sand -> amber, for elevation.
RAMP = np.array([[18, 52, 86], [20, 120, 140], [70, 190, 180], [210, 205, 150], [245, 170, 70]], dtype=np.float32)


def font(size: int, bold: bool = False) -> ImageFont.FreeTypeFont:
    try:
        return ImageFont.truetype(FONT_BOLD if bold else FONT_PATH, size)
    except OSError:
        return ImageFont.load_default()


def ramp(t: np.ndarray) -> np.ndarray:
    t = np.clip(t, 0.0, 1.0) * (len(RAMP) - 1)
    i = np.minimum(t.astype(np.intp), len(RAMP) - 2)
    f = (t - i)[:, None]
    return RAMP[i] * (1 - f) + RAMP[i + 1] * f


@dataclass
class Recording:
    clouds: list[tuple[float, np.ndarray, np.ndarray]] = field(default_factory=list)  # stamp, world xyz, rgb
    poses: list[tuple[float, np.ndarray, np.ndarray]] = field(default_factory=list)  # stamp, position, quat
    sonar: list[tuple[float, SonarImage]] = field(default_factory=list)
    telemetry: list[tuple[float, dict]] = field(default_factory=list)


def load(path: Path, t0: float, t1: float, cloud_stride: int = 1, voxel: float = 0.06) -> Recording:
    rec = Recording()
    tree = TransformTree()
    cloud_index = 0
    for raw in read_recording(path):
        msg = decode(raw)
        stamp = msg.header.stamp if hasattr(msg, "header") else 0.0
        if isinstance(msg, Transform):
            tree.update(msg)
            if msg.child_frame == "base_link" and t0 - 5 <= stamp <= t1 + 5:
                rec.poses.append((stamp, np.array(msg.position), np.array(msg.orientation)))
        elif isinstance(msg, PointCloud):
            if not (t0 <= stamp <= t1):
                continue
            cloud_index += 1
            if cloud_index % cloud_stride:
                continue
            T = tree.lookup(msg.header.frame_id, "map")
            if T is None or msg.num_points == 0:
                continue
            world = transform_points(T, msg.xyz)
            keys = np.floor(world / voxel).astype(np.int64)
            _, keep = np.unique(keys, axis=0, return_index=True)
            rec.clouds.append((stamp, world[keep].astype(np.float32), np.array(msg.rgb[keep])))
        elif isinstance(msg, SonarImage) and t0 - 1 <= stamp <= t1 + 1:
            rec.sonar.append((stamp, msg))
        elif isinstance(msg, Telemetry) and t0 - 1 <= stamp <= t1 + 1:
            rec.telemetry.append((stamp, msg.data))
    rec.clouds.sort(key=lambda c: c[0])
    rec.poses.sort(key=lambda p: p[0])
    return rec


class Map:
    """Global voxel-deduplicated map that grows as clouds are added."""

    def __init__(self, voxel: float) -> None:
        self.voxel = voxel
        self.keys = np.zeros((0,), dtype=np.int64)
        self.xyz = np.zeros((0, 3), dtype=np.float32)
        self.rgb = np.zeros((0, 3), dtype=np.uint8)
        self.born = np.zeros((0,), dtype=np.float32)

    def _key(self, xyz: np.ndarray) -> np.ndarray:
        k = np.floor(xyz / self.voxel).astype(np.int64) + (1 << 19)
        return (k[:, 0] << 40) | (k[:, 1] << 20) | k[:, 2]

    def add(self, xyz: np.ndarray, rgb: np.ndarray, stamp: float) -> None:
        keys = self._key(xyz)
        keys, first = np.unique(keys, return_index=True)
        new = ~np.isin(keys, self.keys, assume_unique=True)
        if not new.any():
            return
        idx = first[new]
        self.keys = np.concatenate([self.keys, keys[new]])
        self.xyz = np.concatenate([self.xyz, xyz[idx]])
        self.rgb = np.concatenate([self.rgb, rgb[idx]])
        self.born = np.concatenate([self.born, np.full(len(idx), stamp, dtype=np.float32)])


def background(w: int, h: int) -> np.ndarray:
    t = np.linspace(0, 1, h, dtype=np.float32)[:, None, None]
    return np.broadcast_to(BG_TOP * (1 - t) + BG_BOTTOM * t, (h, w, 3)).copy()


def look_at(eye: np.ndarray, target: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    f = target - eye
    f = f / np.linalg.norm(f)
    r = np.cross(f, [0.0, 0.0, 1.0])
    r = r / max(np.linalg.norm(r), 1e-9)
    u = np.cross(r, f)
    return r, u, f


def splat(img: np.ndarray, xyz: np.ndarray, colours: np.ndarray, eye: np.ndarray, target: np.ndarray,
          hfov_deg: float = 55.0, size: int = 2, fog_m: float = 70.0) -> None:
    """Draws points into img (H, W, 3 float) with a far-to-near painter's sort and distance fog."""
    h, w, _ = img.shape
    r, u, f = look_at(eye, target)
    d = xyz - eye
    z = d @ f
    ok = z > 0.3
    d, z, col = d[ok], z[ok], colours[ok]
    focal = (w / 2) / math.tan(math.radians(hfov_deg) / 2)
    px = (w / 2 + focal * (d @ r) / z).astype(np.int32)
    py = (h / 2 - focal * (d @ u) / z).astype(np.int32)
    inside = (px >= 0) & (px < w - size) & (py >= 0) & (py < h - size)
    px, py, z, col = px[inside], py[inside], z[inside], col[inside]
    fog = (1.0 - np.exp(-z / fog_m))[:, None]
    bg = img[py, px]
    col = col * (1 - fog) + bg * fog
    order = np.argsort(-z, kind="stable")
    px, py, col = px[order], py[order], col[order]
    for dy in range(size):
        for dx in range(size):
            img[py + dy, px + dx] = col


def project_point(p: np.ndarray, eye: np.ndarray, target: np.ndarray, w: int, h: int, hfov_deg: float = 55.0) -> tuple[float, float] | None:
    r, u, f = look_at(eye, target)
    d = p - eye
    z = float(d @ f)
    if z < 0.3:
        return None
    focal = (w / 2) / math.tan(math.radians(hfov_deg) / 2)
    return w / 2 + focal * float(d @ r) / z, h / 2 - focal * float(d @ u) / z


def finish(img: np.ndarray, glow: float = 0.22) -> Image.Image:
    """Adds a soft bloom so dense point clouds read as surfaces, then converts to 8-bit."""
    base = Image.fromarray(np.clip(img, 0, 255).astype(np.uint8))
    bloom = base.filter(ImageFilter.GaussianBlur(5))
    out = np.asarray(base, dtype=np.float32) + glow * np.asarray(bloom, dtype=np.float32)
    return Image.fromarray(np.clip(out, 0, 255).astype(np.uint8)).filter(ImageFilter.GaussianBlur(0.5))


def colour_map(m: Map, z_lo: float, z_hi: float) -> np.ndarray:
    lum = m.rgb.astype(np.float32).mean(axis=1) / 255.0
    base = ramp((m.xyz[:, 2] - z_lo) / max(z_hi - z_lo, 1e-3))
    return base * (0.55 + 0.6 * lum[:, None])


def pose_at(rec: Recording, t: float) -> tuple[np.ndarray, float]:
    stamps = np.array([p[0] for p in rec.poses])
    i = int(np.clip(np.searchsorted(stamps, t), 0, len(stamps) - 1))
    pos, quat = rec.poses[i][1], rec.poses[i][2]
    fwd = quat_to_matrix(quat)[:3, 0]
    return pos, math.atan2(fwd[1], fwd[0])


def caption(draw: ImageDraw.ImageDraw, w: int, lines: list[tuple[str, int, bool, tuple[int, int, int]]], x: int = 56, y: int = 48) -> None:
    for text, size, bold, colour in lines:
        draw.text((x, y), text, font=font(size, bold), fill=colour)
        y += int(size * 1.35)


def render_build(rec: Recording, out: Path, seconds: float, fps: int, w: int, h: int) -> None:
    t0, t1 = rec.clouds[0][0], rec.clouds[-1][0]
    frames = int(seconds * fps)
    zs = np.concatenate([c[1][:, 2] for c in rec.clouds[:: max(1, len(rec.clouds) // 20)]])
    z_lo, z_hi = np.percentile(zs, [2, 99.5])
    m = Map(0.07)
    ci = 0
    smooth_pos, smooth_yaw = None, None
    trail: list[np.ndarray] = []
    for k in range(frames):
        t = t0 + (t1 - t0) * k / max(frames - 1, 1)
        while ci < len(rec.clouds) and rec.clouds[ci][0] <= t:
            stamp, xyz, rgb = rec.clouds[ci]
            m.add(xyz, rgb, stamp)
            ci += 1
        pos, yaw = pose_at(rec, t)
        if smooth_pos is None:
            smooth_pos, smooth_yaw = pos.copy(), yaw
        smooth_pos += (pos - smooth_pos) * 0.12
        smooth_yaw += math.atan2(math.sin(yaw - smooth_yaw), math.cos(yaw - smooth_yaw)) * 0.05
        trail.append(pos.copy())

        # Camera behind, above and off to the side of the vehicle, looking slightly ahead of it.
        back = np.array([math.cos(smooth_yaw), math.sin(smooth_yaw), 0.0])
        side = np.array([-back[1], back[0], 0.0])
        target = smooth_pos + back * 4.0 + np.array([0, 0, -1.2])
        eye = smooth_pos - back * 13.0 + side * 7.0 + np.array([0, 0, 8.0])

        img = background(w, h)
        if len(m.xyz):
            col = colour_map(m, z_lo, z_hi)
            fresh = np.clip(1.0 - (t - m.born) / 1.2, 0.0, 1.0)[:, None]
            col = col * (1 - fresh) + np.array([235, 255, 255], dtype=np.float32) * fresh
            splat(img, m.xyz, col, eye, target, size=2)
        frame = finish(img)
        draw = ImageDraw.Draw(frame)
        pts = [project_point(p, eye, target, w, h) for p in trail[::3]]
        pts = [p for p in pts if p is not None]
        if len(pts) > 1:
            draw.line(pts, fill=(255, 150, 60), width=3)
        here = project_point(pos, eye, target, w, h)
        if here:
            x, y = here
            draw.ellipse([x - 9, y - 9, x + 9, y + 9], outline=(255, 170, 70), width=3)
            draw.ellipse([x - 3, y - 3, x + 3, y + 3], fill=(255, 200, 120))
        caption(draw, w, [
            ("DEPTH CAMERA  ->  3D MAP", 30, True, (220, 245, 245)),
            (f"{len(m.xyz):,} points   |   {np.linalg.norm(trail[-1][:2] - trail[0][:2]):.0f} m surveyed   |   sim t = {t - t0:5.1f} s", 22, False, (140, 190, 195)),
        ])
        frame.save(out / f"frame_{k:05d}.png")
        if k % 30 == 0:
            print(f"build {k}/{frames}  points {len(m.xyz):,}", flush=True)


def render_orbit(rec: Recording, out: Path, seconds: float, fps: int, w: int, h: int, degrees: float) -> None:
    """Low flyover along the finished map, following the vehicle's track."""
    m = Map(0.05)
    for stamp, xyz, rgb in rec.clouds:
        m.add(xyz, rgb, stamp)
    z_lo, z_hi = np.percentile(m.xyz[:, 2], [2, 99.5])
    col = colour_map(m, z_lo, z_hi)
    traj = np.array([p[1] for p in rec.poses])
    # Arc-length parameterise the (smoothed) track so the camera moves at constant speed.
    kernel = np.ones(25) / 25
    smooth = np.stack([np.convolve(traj[:, i], kernel, mode="valid") for i in range(3)], axis=1)
    seg = np.linalg.norm(np.diff(smooth[:, :2], axis=0), axis=1)
    arc = np.concatenate([[0.0], np.cumsum(seg)])
    total = arc[-1]
    frames = int(seconds * fps)
    lead, back, height, side = 10.0, 16.0, 9.0, 5.0
    for k in range(frames):
        s = lead + back + (total - 2 * (lead + back)) * k / max(frames - 1, 1)
        at = lambda d: np.array([np.interp(d, arc, smooth[:, i]) for i in range(3)])
        here, ahead, behind = at(s), at(s + lead), at(s - back)
        direction = (ahead - behind)[:2]
        direction = direction / max(np.linalg.norm(direction), 1e-6)
        lateral = np.array([-direction[1], direction[0], 0.0])
        eye = behind + lateral * side + np.array([0, 0, height])
        target = ahead + np.array([0, 0, -2.5])
        near = np.sum((m.xyz[:, :2] - eye[:2]) ** 2, axis=1) < 75.0 ** 2
        img = background(w, h)
        splat(img, m.xyz[near], col[near], eye, target, hfov_deg=60.0, size=2, fog_m=45.0)
        frame = finish(img)
        draw = ImageDraw.Draw(frame)
        caption(draw, w, [
            ("SURVEY MAP", 30, True, (220, 245, 245)),
            (f"{len(m.xyz):,} points at 5 cm  |  fused from {len(rec.clouds)} depth-camera clouds  |  {total:.0f} m of pipeline", 22, False, (140, 190, 195)),
        ])
        frame.save(out / f"frame_{k:05d}.png")
        if k % 30 == 0:
            print(f"flyover {k}/{frames}", flush=True)
    stats = {"map_points": int(len(m.xyz)), "clouds": len(rec.clouds), "track_m": round(float(total), 1),
             "orbit_caption": f"{len(m.xyz) / 1e6:.1f} million points fused from {len(rec.clouds)} depth-camera clouds over {total:.0f} m"}
    (out.parent / "stats.json").write_text(json.dumps(stats, indent=2))

def sparkline(draw: ImageDraw.ImageDraw, box: tuple[int, int, int, int], values: list[float], vmax: float, colour: tuple[int, int, int], label: str, unit: str) -> None:
    x0, y0, x1, y1 = box
    draw.rectangle(box, outline=(40, 70, 80), width=1)
    draw.text((x0 + 10, y0 + 6), label, font=font(20, True), fill=colour)
    if values:
        draw.text((x1 - 10, y0 + 6), f"{values[-1]:,.0f}{unit}", font=font(20, True), fill=colour, anchor="ra")
    if len(values) > 1:
        n = len(values)
        pts = [(x0 + 8 + (x1 - x0 - 16) * i / (n - 1), y1 - 10 - (y1 - y0 - 44) * min(v / vmax, 1.0)) for i, v in enumerate(values)]
        draw.line(pts, fill=colour, width=3)


def render_sonar(rec: Recording, out: Path, meta: dict, w: int, h: int) -> None:
    start, dt, frames = meta["start_sim_time"], meta["fixed_dt"], meta["frames"]
    sonar_stamps = np.array([s[0] for s in rec.sonar])
    tel = rec.telemetry
    tel_stamps = np.array([s for s, _ in tel])
    pts_hist: list[float] = []
    sonar_hist: list[float] = []
    max_pts = max((d.get("points", 0) for _, d in tel), default=1)
    for k in range(frames):
        t = start + k * dt
        i = int(np.clip(np.searchsorted(sonar_stamps, t, side="right") - 1, 0, len(rec.sonar) - 1))
        sonar = rec.sonar[i][1]
        j = int(np.clip(np.searchsorted(tel_stamps, t, side="right") - 1, 0, len(tel) - 1))
        data = tel[j][1]
        pts_hist.append(float(data.get("points", 0)))
        sonar_hist.append(float(np.mean(sonar.intensity)) / 255.0 * 100.0)

        panel = Image.new("RGB", (w, h), (3, 8, 12))
        draw = ImageDraw.Draw(panel)
        fan = Image.fromarray(render_fan(sonar, width=w - 60))
        panel.paste(fan, (30, 150))
        caption(draw, w, [
            ("IMAGING SONAR", 30, True, (240, 200, 140)),
            (f"256 beams  |  120 deg  |  {sonar.max_range:.0f} m  |  turbidity {data.get('turbidity', 0):.2f}", 21, False, (170, 150, 120)),
        ], x=30, y=40)
        top = 150 + fan.height + 40
        hh = (h - top - 60) // 2
        sparkline(draw, (30, top, w - 30, top + hh), pts_hist, max_pts * 1.1, (120, 230, 230), "DEPTH CAMERA RETURNS", " pts")
        sparkline(draw, (30, top + hh + 20, w - 30, top + 2 * hh + 20), sonar_hist, 60.0, (245, 180, 90), "SONAR ECHO LEVEL", " %")
        panel.save(out / f"frame_{k:05d}.png")
        if k % 30 == 0:
            print(f"sonar {k}/{frames}", flush=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("mode", choices=["build", "orbit", "sonar"])
    parser.add_argument("ssim", type=Path)
    parser.add_argument("out", type=Path)
    parser.add_argument("--t0", type=float, default=0.0)
    parser.add_argument("--t1", type=float, default=1e9)
    parser.add_argument("--seconds", type=float, default=10.0)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--width", type=int, default=1920)
    parser.add_argument("--height", type=int, default=1080)
    parser.add_argument("--degrees", type=float, default=150.0)
    parser.add_argument("--stride", type=int, default=1, help="use every Nth cloud")
    parser.add_argument("--meta", type=Path, help="meta.json of the game recording to sync to (sonar mode)")
    args = parser.parse_args()

    if args.mode == "sonar":
        meta = json.loads(args.meta.read_text())
        args.t0 = meta["start_sim_time"] - 2
        args.t1 = meta["start_sim_time"] + meta["frames"] * meta["fixed_dt"] + 2
    rec = load(args.ssim, args.t0, args.t1, cloud_stride=args.stride if args.mode != "sonar" else 1000000)
    print(f"loaded {len(rec.clouds)} clouds, {len(rec.poses)} poses, {len(rec.sonar)} sonar, {len(rec.telemetry)} telemetry", flush=True)
    args.out.mkdir(parents=True, exist_ok=True)
    if args.mode == "build":
        render_build(rec, args.out, args.seconds, args.fps, args.width, args.height)
    elif args.mode == "orbit":
        render_orbit(rec, args.out, args.seconds, args.fps, args.width, args.height, args.degrees)
    else:
        render_sonar(rec, args.out, meta, args.width, args.height)
    return 0


if __name__ == "__main__":
    sys.exit(main())
