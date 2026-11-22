#!/usr/bin/env bash
# Dump the individual PolygonStop returns from a robot already parked in
# the pose under diagnosis. Needs only the simulator.
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/env.sh"
WT="$(cd "$HERE/.." && pwd)"
mkdir -p "$HERE/results"
python3 "$WT/docs/data/c2nav6_stopgeom.py" "$HERE/results/${1:-geom}_geom.json"
