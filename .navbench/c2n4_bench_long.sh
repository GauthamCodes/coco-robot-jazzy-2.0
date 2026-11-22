#!/usr/bin/env bash
# C2-NAV.4: c2n4_bench.sh with a 150 s leg budget instead of 75 s.
# Everything else identical. See c2n4_long.sh for why.
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/env.sh"
WT="$(cd "$HERE/.." && pwd)"
PARAMS="$1"
TAG="${2:-c2n4long}"
mkdir -p "$HERE/results" "$HERE/logs"

bash "$WT/gazebo_models/scripts/ros_clean.sh" >/dev/null 2>&1
sleep 5
bash "$HERE/up_sim.sh" || exit 1
sleep 3
bash "$HERE/c2n4_up_nav.sh" "$PARAMS" "$TAG" || exit 1
bash "$HERE/wait_nav.sh" | head -1
sleep 3
bash "$HERE/c2n4_verify.sh" "$TAG" | grep -E 'cost_scaling|BaseObstacle.scale|goal_checker.plugin|inflation_radius'
echo "=== bench, 150 s budget ==="
python3 "$WT/gazebo_models/scripts/nav_bench.py" \
    --tag "$TAG" --repeats 1 --timeout 150 --only enclosure_entry \
    --out "$HERE/results"
echo "bench exit $?"
