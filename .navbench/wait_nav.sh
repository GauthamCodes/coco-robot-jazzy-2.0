#!/usr/bin/env bash
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/env.sh"
NODES="/bt_navigator /controller_server /planner_server /amcl /map_server /local_costmap/local_costmap /global_costmap/global_costmap /velocity_smoother /collision_monitor /behavior_server"
for i in $(seq 1 120); do
    ok=1
    for n in $NODES; do
        st=$(timeout 5 ros2 lifecycle get "$n" 2>/dev/null | awk '{print $1}')
        [ "$st" = "active" ] || ok=0
    done
    [ "$ok" = "1" ] && { echo "ALL ACTIVE after ${i} polls"; break; }
    sleep 2
done
echo "--- final states ---"
for n in $NODES; do
    printf '%-40s %s\n' "$n" "$(timeout 5 ros2 lifecycle get "$n" 2>/dev/null)"
done
