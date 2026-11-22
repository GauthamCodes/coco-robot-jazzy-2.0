#!/usr/bin/env bash
# up_nav.sh [arbiter]   -- arbiter = "true" starts arbiter.launch.py too
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/env.sh"
mkdir -p "$HERE/logs"
ARB="${1:-false}"
if [ "$ARB" = "true" ]; then
    ros2 launch custom_teleop arbiter.launch.py > "$HERE/logs/arbiter.log" 2>&1 &
    echo "arbiter pid $!"
    sleep 3
fi
ros2 launch gazebo_models nav.launch.py arbiter:="$ARB" \
    > "$HERE/logs/nav.log" 2>&1 &
echo "nav pid $!"
for i in $(seq 1 120); do
    st=$(timeout 5 ros2 lifecycle get /bt_navigator 2>/dev/null)
    if echo "$st" | grep -q active; then echo "nav2 active after ${i}s"; exit 0; fi
    sleep 1
done
echo "NAV2 DID NOT ACTIVATE"; tail -40 "$HERE/logs/nav.log"; exit 1
