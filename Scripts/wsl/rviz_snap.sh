#!/usr/bin/env bash
# Save a PNG of the RViz2 window:  rviz_snap.sh /mnt/e/.../out.png
# Needs x11-utils/imagemagick/xdotool (installed on first use). RViz2 must run on X11 (xcb).
set -eo pipefail
out="${1:?usage: rviz_snap.sh out.png}"
if ! command -v xdotool >/dev/null || ! command -v import >/dev/null; then
    DEBIAN_FRONTEND=noninteractive apt-get install -y -qq xdotool imagemagick x11-utils >/dev/null
fi
export DISPLAY=${DISPLAY:-:0}
win=$(xdotool search --name "RViz" | tail -n 1)
[ -n "$win" ] || { echo "no RViz window" >&2; exit 1; }
import -window "$win" "$out"
echo "saved $out ($(identify -format '%wx%h' "$out"))"
