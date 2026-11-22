#!/usr/bin/env bash
# C2-NAV.4 behaviour run: ONE fresh enclosure_entry leg through
# nav_bench.py -- the same instrument C2-NAV.0, .1 and .2 used, so the
# numbers land in the same table.
#
#   c2n4_bench.sh <abs params path> <tag>
#
# --repeats 1, deliberately: repeat 2 and 3 start from wherever repeat 1
# ended, which at the baseline is the stall itself. They are escape
# probes, not independent trials, and C2-NAV.2 recorded that.
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/env.sh"
WT="$(cd "$HERE/.." && pwd)"
PARAMS="$1"
TAG="${2:-c2n4bench}"
mkdir -p "$HERE/results" "$HERE/logs"

bash "$WT/gazebo_models/scripts/ros_clean.sh" >/dev/null 2>&1
sleep 5
bash "$HERE/up_sim.sh" || exit 1
sleep 3
bash "$HERE/c2n4_up_nav.sh" "$PARAMS" "$TAG" || exit 1
bash "$HERE/wait_nav.sh" | head -1
sleep 3
bash "$HERE/c2n4_verify.sh" "$TAG" | grep -E 'cost_scaling|BaseObstacle.scale|goal_checker.plugin|inflation_radius'
echo "=== bench ==="
python3 "$WT/gazebo_models/scripts/nav_bench.py" \
    --tag "$TAG" --repeats 1 --timeout 75 --only enclosure_entry \
    --out "$HERE/results"
echo "bench exit $?"
