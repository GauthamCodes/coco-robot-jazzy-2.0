#!/usr/bin/env bash
# C2-NAV.1 section 12: WATCH one post-change enclosure_entry leg.
# Attaches the Gazebo GUI client to the already-running server, records
# the X display while nav_bench drives the single leg, and drops frames
# every 2 s so the behaviour can be looked at rather than inferred.
# Run AFTER the measured tour: the GUI adds load and must not touch the
# numbers.
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/env.sh"
SHOT="$HERE/observe"
rm -rf "$SHOT"; mkdir -p "$SHOT"

gz sim -g > "$HERE/logs/gzgui.log" 2>&1 &
GUI=$!
echo "gz gui pid $GUI"
sleep 25

ffmpeg -y -f x11grab -framerate 10 -video_size 1920x1080 -i :1 \
    -t 100 -vf fps=1/2 "$SHOT/frame_%03d.png" \
    > "$HERE/logs/ffmpeg.log" 2>&1 &
FF=$!
echo "ffmpeg pid $FF"
sleep 2

python3 "$HERE/../gazebo_models/scripts/nav_bench.py" \
    --tag observe_enclosure --repeats 1 --timeout 75 \
    --only enclosure_entry --out "$HERE/results"

wait $FF 2>/dev/null
kill $GUI 2>/dev/null
ls -la "$SHOT" | tail -5
