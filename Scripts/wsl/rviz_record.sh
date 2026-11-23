#!/usr/bin/env bash
# Record the RViz2 window to an MP4:  rviz_record.sh out.mp4 [seconds] [fps]
set -eo pipefail
out="${1:?usage: rviz_record.sh out.mp4 [seconds] [fps]}"
secs="${2:-20}"
fps="${3:-30}"
command -v ffmpeg >/dev/null || DEBIAN_FRONTEND=noninteractive apt-get install -y -qq ffmpeg >/dev/null
export DISPLAY=${DISPLAY:-:0}
win=$(xdotool search --name "RViz" | tail -n 1)
[ -n "$win" ] || { echo "no RViz window" >&2; exit 1; }
ffmpeg -y -loglevel error -f x11grab -draw_mouse 0 -window_id "$win" -framerate "$fps" -t "$secs" -i "$DISPLAY" \
    -vf "crop=trunc(iw/2)*2:trunc(ih/2)*2" -c:v libx264 -preset slow -crf 20 -pix_fmt yuv420p \
    -movflags +faststart "$out"
echo "saved $out"
