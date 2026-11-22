#!/usr/bin/env bash
# Per-process CPU while a leg is running. controller_server is the one
# that matters: DWB samples vx_samples x vtheta_samples trajectories every
# control period, and if that does not fit in 100 ms the 10 Hz
# controller_frequency is aspirational.
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/env.sh"
NCPU=$(nproc)
echo "cores: $NCPU"
for name in controller_server planner_server gz "ruby" nav_bench \
            collision_monitor velocity_smoother amcl; do
    pids=$(pgrep -f "$name" | head -3 | tr '\n' ' ')
    [ -z "$pids" ] && continue
    for p in $pids; do
        c=$(ps -o %cpu= -p "$p" 2>/dev/null | tr -d ' ')
        cmd=$(ps -o comm= -p "$p" 2>/dev/null)
        [ -n "$c" ] && printf '%-22s pid %-8s %%cpu %-7s (%s)\n' \
            "$name" "$p" "$c" "$cmd"
    done
done
echo "--- 5 s average of the top consumers ---"
top -b -n 2 -d 5 | awk '/^ *PID/{h=1;next} h&&NF>8{print $9"  "$12}' \
    | tail -n +1 | sort -rn | head -8
