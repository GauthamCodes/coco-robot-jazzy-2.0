#!/usr/bin/env bash
# C2-NAV.1: same as up_nav.sh, but points nav.launch.py at the WORKTREE
# nav2_params.yaml. The installed share/ symlinks to the trunk checkout,
# so without params_file:= the edited goal checker would never be read.
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/env.sh"
mkdir -p "$HERE/logs"
PARAMS="$HERE/../gazebo_models/config/nav2_params.yaml"
echo "params_file: $PARAMS"
grep -n 'GoalChecker' "$PARAMS"
ros2 launch gazebo_models nav.launch.py arbiter:=false \
    params_file:="$PARAMS" > "$HERE/logs/nav_c2nav1.log" 2>&1 &
echo "nav pid $!"
for i in $(seq 1 150); do
    st=$(timeout 5 ros2 lifecycle get /bt_navigator 2>/dev/null)
    if echo "$st" | grep -q active; then echo "nav2 active after ${i}s"; exit 0; fi
    sleep 1
done
echo "NAV2 DID NOT ACTIVATE"; tail -40 "$HERE/logs/nav_c2nav1.log"; exit 1
