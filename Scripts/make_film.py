"""Assembles recorded takes and sensor renders into the case-study film, web clips and stills.

Inputs: Saved/Recordings/<take>/frame_#####.png (from Scripts/shots/film.json) and
Saved/Film/renders/<name>/frame_#####.png (from render_sensor_video.py).
Outputs go to Saved/Film/out.

    Bridge\\.venv\\Scripts\\python.exe Scripts\\make_film.py
"""

from __future__ import annotations

import json
import shutil
import subprocess
import sys
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

PROJECT = Path(__file__).resolve().parent.parent
TAKES = PROJECT / "Saved" / "Recordings"
RENDERS = PROJECT / "Saved" / "Film" / "renders"
OUT = PROJECT / "Saved" / "Film" / "out"
WORK = PROJECT / "Saved" / "Film" / "work"
FPS = 30
FADE = 0.5
FONT = "C\\:/Windows/Fonts/segoeui.ttf"
FONT_BOLD = "C\\:/Windows/Fonts/segoeuib.ttf"


def run(args: list[str]) -> None:
    result = subprocess.run(["ffmpeg", "-y", "-loglevel", "error", *args], capture_output=True, text=True)
    if result.returncode != 0:
        raise RuntimeError(f"ffmpeg failed: {' '.join(args)}\n{result.stderr}")


def source(name: str) -> Path:
    for root in (TAKES, RENDERS, WORK):
        if (root / name).is_dir():
            return root / name
    raise FileNotFoundError(name)


def x264(crf: int) -> list[str]:
    return ["-c:v", "libx264", "-preset", "slow", "-crf", str(crf), "-pix_fmt", "yuv420p", "-movflags", "+faststart", "-an"]


def captioned(name: str, start: float, seconds: float, title: str, sub: str, out: Path) -> None:
    """One 1080p segment with a fading lower-third caption."""
    folder = source(name)
    fade_in = "if(lt(t,0.5),t/0.5,if(gt(t,{d}-0.6),max(0,({d}-0.2-t)/0.4),1))".format(d=seconds)
    title_file = WORK / f"{out.stem}_title.txt"
    sub_file = WORK / f"{out.stem}_sub.txt"
    title_file.write_text(title, encoding="utf-8")
    sub_file.write_text(sub, encoding="utf-8")
    rel = lambda p: str(p).replace("\\", "/").replace(":", "\\:")
    vf = [
        "scale=1920:1080:flags=lanczos",
        # Soft gradient behind the caption so it reads over bright frames.
        "drawbox=x=0:y=ih-230:w=iw:h=230:color=black@0.38:t=fill",
    ]
    if title:
        vf.append(f"drawtext=fontfile='{FONT_BOLD}':textfile='{rel(title_file)}':fontsize=46:fontcolor=white:x=80:y=h-178:alpha='{fade_in}'")
    if sub:
        vf.append(f"drawtext=fontfile='{FONT}':textfile='{rel(sub_file)}':fontsize=29:fontcolor=0xBFE6E6:x=82:y=h-108:alpha='{fade_in}'")
    if not title and not sub:
        vf = vf[:1]
    run(["-framerate", str(FPS), "-start_number", str(int(start * FPS)), "-i", str(folder / "frame_%05d.png"),
         "-frames:v", str(int(seconds * FPS)), "-vf", ",".join(vf), "-r", str(FPS), *x264(16), str(out)])


def plain(name: str, start: float, seconds: float, out: Path, width: int, crf: int) -> None:
    folder = source(name)
    run(["-framerate", str(FPS), "-start_number", str(int(start * FPS)), "-i", str(folder / "frame_%05d.png"),
         "-frames:v", str(int(seconds * FPS)), "-vf", f"scale={width}:-2:flags=lanczos", "-r", str(FPS), *x264(crf), str(out)])


def still(name: str, frame: int, out: Path, width: int = 1920, quality: int = 86) -> None:
    img = Image.open(source(name) / f"frame_{frame:05d}.png").convert("RGB")
    if img.width != width:
        img = img.resize((width, round(img.height * width / img.width)), Image.LANCZOS)
    img.save(out, quality=quality, optimize=True, progressive=True)


def side_by_side(left: str, right: str, out_dir: Path) -> None:
    """Centre half of the onboard camera on the left, sonar panel on the right, frame by frame."""
    out_dir.mkdir(parents=True, exist_ok=True)
    lf, rf = sorted(source(left).glob("frame_*.png")), sorted(source(right).glob("frame_*.png"))
    for i, (a, b) in enumerate(zip(lf, rf)):
        canvas = Image.new("RGB", (1920, 1080))
        cam = Image.open(a).convert("RGB")
        canvas.paste(cam.crop((480, 0, 1440, 1080)), (0, 0))
        canvas.paste(Image.open(b).convert("RGB").resize((960, 1080)), (960, 0))
        d = ImageDraw.Draw(canvas)
        d.line([(960, 0), (960, 1080)], fill=(20, 60, 70), width=2)
        d.text((40, 40), "ONBOARD CAMERA", font=ImageFont.truetype("C:/Windows/Fonts/segoeuib.ttf", 30), fill=(220, 245, 245))
        canvas.save(out_dir / f"frame_{i:05d}.png")


def end_card(out_dir: Path, seconds: float, lines: list[tuple[str, int, bool, tuple[int, int, int]]]) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    img = Image.new("RGB", (1920, 1080), (3, 9, 14))
    d = ImageDraw.Draw(img)
    y = 400
    for text, size, bold, colour in lines:
        f = ImageFont.truetype("C:/Windows/Fonts/segoeuib.ttf" if bold else "C:/Windows/Fonts/segoeui.ttf", size)
        d.text((960, y), text, font=f, fill=colour, anchor="mm")
        y += int(size * 1.6)
    for i in range(int(seconds * FPS)):
        img.save(out_dir / f"frame_{i:05d}.png")


def xfade_concat(segments: list[tuple[Path, float]], out: Path, crf: int) -> None:
    inputs: list[str] = []
    for path, _ in segments:
        inputs += ["-i", str(path)]
    chain, offset, last = [], 0.0, "[0:v]"
    for i in range(1, len(segments)):
        offset += segments[i - 1][1] - FADE
        label = f"[v{i}]"
        chain.append(f"{last}[{i}:v]xfade=transition=fade:duration={FADE}:offset={offset:.3f}{label}")
        last = label
    chain.append(f"{last}fade=t=in:st=0:d=0.8,fade=t=out:st={offset + segments[-1][1] - 1.0:.3f}:d=1.0[out]")
    run([*inputs, "-filter_complex", ";".join(chain), "-map", "[out]", "-r", str(FPS), *x264(crf), str(out)])


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)
    WORK.mkdir(parents=True, exist_ok=True)
    stats = json.loads((RENDERS / "stats.json").read_text()) if (RENDERS / "stats.json").exists() else {}

    film_only = "--film-only" in sys.argv
    if not film_only:
        print("compositing camera vs sonar", flush=True)
        side_by_side("G_turbid", "sonar_turbid", WORK / "sonar_vs_camera")
    end_card(WORK / "end_card", 4.0, [
        ("SubseaSim", 92, True, (230, 248, 248)),
        ("Underwater robot digital twin", 38, False, (150, 200, 205)),
        ("C++  ·  Unreal Engine 5.8  ·  Python  ·  point clouds  ·  imaging sonar", 28, False, (110, 160, 168)),
        ("Neil Lasrado  ·  2026", 30, False, (240, 180, 100)),
    ])

    segments = [
        ("E2_orbit", 1.0, 7.0, "SubseaSim", "An underwater robot digital twin in Unreal Engine 5.8, written in C++"),
        ("D2_flyby", 4.5, 6.0, "6-DOF hydrodynamics", "Buoyancy, drag, added mass and ocean current on a Chaos rigid body"),
        ("B2_chase_hud", 1.0, 8.0, "Autonomous pipeline inspection", "Line-of-sight guidance, PID control and eight-thruster allocation"),
        ("C_fpv", 0.0, 5.5, "Onboard camera", "Lights, silt, marine snow and a procedural seabed"),
        ("build", 0.0, 8.0, "Depth camera to live 3D point cloud", "Built off the game thread and streamed to Python and ROS tooling"),
        ("F2_fault", 0.0, 7.5, "Fault injection", "Thruster 1 fails and thrust is re-solved across the other seven"),
        ("sonar_vs_camera", 0.0, 11.0, "Silt rolls in", "The camera loses half its returns. The imaging sonar does not"),
        ("A_wide", 1.5, 6.0, "Built to be tested", "Automated tests, scripted fault scenarios and headless benchmarks"),
        ("I2_manifold", 2.0, 8.0, "Procedural inspection site", "Seabed, pipeline, free spans, debris and a manifold from one seed"),
        ("orbit", 0.0, 7.0, "Survey map", stats.get("orbit_caption", "Fused from the depth camera as the ROV flies")),
        ("end_card", 0.0, 4.0, "", ""),
    ]
    parts = []
    for i, (name, start, seconds, title, sub) in enumerate(segments):
        print(f"segment {i}: {name}", flush=True)
        path = WORK / f"seg_{i:02d}.mp4"
        captioned(name, start, seconds, title, sub, path)
        parts.append((path, seconds))
    print("film", flush=True)
    xfade_concat(parts, OUT / "final.mp4", crf=23)
    if film_only:
        return 0

    print("web clips", flush=True)
    plain("E2_orbit", 0.0, 9.0, OUT / "loop.mp4", 1280, 27)
    clips = {
        "autopilot-hud": ("B2_chase_hud", 0.5, 9.0, 60),
        "onboard": ("C_fpv", 0.0, 7.0, 60),
        "map-build": ("build", 0.0, 10.0, 150),
        "fault": ("F2_fault", 0.0, 8.5, 150),
        "sonar-vs-camera": ("sonar_vs_camera", 0.0, 11.0, 300),
        "map-orbit": ("orbit", 0.0, 10.0, 120),
        "flyby": ("D2_flyby", 0.5, 10.5, 240),
        "manifold": ("I2_manifold", 1.0, 11.0, 240),
    }
    for clip, (name, start, seconds, poster_frame) in clips.items():
        plain(name, start, seconds, OUT / f"{clip}.mp4", 1280, 26)
        still(name, poster_frame, OUT / f"{clip}.jpg", width=1280, quality=82)

    print("stills", flush=True)
    gallery = OUT / "gallery"
    gallery.mkdir(exist_ok=True)
    for i, (name, frame) in enumerate([("A_wide", 120), ("C_fpv", 120), ("D2_flyby", 260), ("E2_orbit", 150),
                                       ("I2_manifold", 270), ("B2_chase_hud", 150), ("J_top", 90), ("H_side", 100)], start=1):
        still(name, frame, gallery / f"{i:02d}.jpg")
    still("E2_orbit", 120, OUT / "cover.jpg")
    for f in sorted(OUT.glob("*.mp4")):
        print(f"{f.name:24s} {f.stat().st_size / 1e6:6.1f} MB")
    return 0


if __name__ == "__main__":
    sys.exit(main())
