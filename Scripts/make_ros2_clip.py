"""Compose the Unreal + RViz2 recordings (Scripts/wsl/record_ros2_demo.ps1) into the ROS 2 clip.

    python Scripts/make_ros2_clip.py [--src Saved/ROS2] [--out Saved/ROS2]

Writes ros2-rviz.mp4 (1920x1080), ros2-rviz-web.mp4 (1280x720, for the site), a poster and a still.
"""

from __future__ import annotations

import argparse
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
FONT = "C\\:/Windows/Fonts/segoeui.ttf"
FONT_BOLD = "C\\:/Windows/Fonts/segoeuib.ttf"
BG = "0x06111A"
CAPTION = "Same run, live: Unreal streams point clouds, sonar and pose to ROS 2. RViz2 shows them."


def run(args: list[str], cwd: Path | None = None) -> None:
    subprocess.run(args, check=True, cwd=cwd)


def text_file(folder: Path, name: str, text: str) -> str:
    """drawtext reads text from a file, which sidesteps filter-graph escaping of : , and '."""
    (folder / name).write_text(text, encoding="utf-8")
    return name


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", type=Path, default=ROOT / "Saved" / "ROS2")
    ap.add_argument("--out", type=Path, default=ROOT / "Saved" / "ROS2")
    ap.add_argument("--seconds", type=float, default=22.0)
    ns = ap.parse_args()
    ns.out.mkdir(parents=True, exist_ok=True)
    unreal, rviz = ns.src / "unreal.mp4", ns.src / "rviz.mp4"
    full = ns.out / "ros2-rviz.mp4"

    # Two 944 px wide panes on a 1920x1080 canvas, labels above, caption below.
    w = 944
    top = 250  # panes are ~566 px tall: this centres labels + panes + caption on the canvas
    label = f"fontfile='{FONT_BOLD}':fontsize=34:fontcolor=white:y={top - 58}"
    sub = f"fontfile='{FONT}':fontsize=24:fontcolor=0x8FC9C9:y={top - 22}"
    fade = f"fade=t=in:st=0:d=0.6,fade=t=out:st={ns.seconds - 0.8}:d=0.8"
    t = {k: text_file(ns.out, f"_ros2_{k}.txt", v) for k, v in {
        "l1": "Unreal Engine 5.8", "s1": "the simulation (Windows)",
        "l2": "ROS 2 Jazzy · RViz2", "s2": "robotics tools (Ubuntu in WSL)", "cap": CAPTION}.items()}
    graph = (
        f"[0:v]scale={w}:-2,setsar=1[ue];"
        f"[1:v]scale={w}:-2,setsar=1[rv];"
        f"color=c={BG}:s=1920x1080:r=30:d={ns.seconds}[bg];"
        f"[bg][ue]overlay=x=12:y={top}[a];"
        f"[a][rv]overlay=x={1920 - w - 12}:y={top}:shortest=1,"
        f"drawtext={label}:x=14:textfile={t['l1']},"
        f"drawtext={sub}:x=16:textfile={t['s1']},"
        f"drawtext={label}:x={1920 - w - 10}:textfile={t['l2']},"
        f"drawtext={sub}:x={1920 - w - 8}:textfile={t['s2']},"
        f"drawtext=fontfile='{FONT}':fontsize=30:fontcolor=0xBFE6E6:x=(w-tw)/2:y={top + 610}:textfile={t['cap']},"
        f"{fade},format=yuv420p[v]"
    )
    run(["ffmpeg", "-y", "-loglevel", "error", "-i", str(unreal), "-i", str(rviz),
         "-filter_complex", graph, "-map", "[v]", "-t", str(ns.seconds), "-r", "30",
         "-c:v", "libx264", "-preset", "slow", "-crf", "19", "-movflags", "+faststart", str(full)],
        cwd=ns.out)
    for name in t.values():
        (ns.out / name).unlink()

    web = ns.out / "ros2-rviz-web.mp4"
    run(["ffmpeg", "-y", "-loglevel", "error", "-i", str(full), "-vf", "scale=1280:720",
         "-c:v", "libx264", "-preset", "slow", "-crf", "24", "-movflags", "+faststart", str(web)])
    run(["ffmpeg", "-y", "-loglevel", "error", "-ss", "1.2", "-i", str(full), "-frames:v", "1",
         "-vf", "scale=1280:720", "-q:v", "3", str(ns.out / "ros2-rviz-poster.jpg")])
    run(["ffmpeg", "-y", "-loglevel", "error", "-ss", str(ns.seconds * 0.6), "-i", str(full),
         "-frames:v", "1", "-q:v", "2", str(ns.out / "ros2-rviz-still.jpg")])
    for f in (full, web):
        print(f"{f.name}: {f.stat().st_size / 1e6:.1f} MB")


if __name__ == "__main__":
    main()
