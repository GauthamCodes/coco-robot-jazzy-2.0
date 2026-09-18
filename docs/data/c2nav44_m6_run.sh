#!/usr/bin/env bash
# C2-NAV.44: ONE nominal M6 fetch on the fixed command path (c2nav43-integration).
#   fresh headless sim (traverse:=true) -> mission.launch.py rviz:=false
#   target_colour:=$COLOUR (every other argument at its shipped default:
#   executive on, target_source target_finder, web on, localization monitor +
#   recovery on, depth_cloud not passed = off)
#   bring-up checks: Nav2 lifecycle active, params readback vs the shipped
#     file, mission nodes, exactly one arbiter, verify-topology B (every
#     command-chain link), depth fusion off, no process died
#   then /mission/start and wait for COMPLETE or ABORT, recorded end to end by
#   c2nav42_cmdpath.py record (command chain + collision monitor + mission
#   state + ground truth) and c2m51_hrec.py (localization health + state),
#   plus the raw /mission/state stream.
# Adapted from C2-NAV.42's live_mission.sh with the class-A injection REMOVED.
set -o pipefail
WT="${COCO_WT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
# The COLCON WORKSPACE root, which is NOT the repo root: this repo lives at
# <ws>/src/coco-robot-ros2, so the overlay is <ws>/install. These scripts were
# written in a worktree that carried its own install/ and build/, and using
# "$WT/install" on main sources a STALE <repo>/install instead (C2-NAV.48).
# Derived the way setup_env.sh derives it, and overridable the same way $WT is.
WS="${COCO_WS:-$(cd "$WT/../.." && pwd)}"
OUT="${1:?usage: m6_run.sh OUT_DIR colour}"
COLOUR="${2:?usage: m6_run.sh OUT_DIR colour}"
MISSION_BUDGET=900
mkdir -p "$OUT"
exec > >(tee -a "$OUT/runner.log") 2>&1
say() { echo "m6_run: $* ($(date -u +%H:%M:%S) UTC)"; }
FAIL=0
COMPLETED=0
check() { if "$@"; then say "PASS $CHECK"; else say "FAIL $CHECK"; FAIL=1; fi; }

for p in 'g[z] sim' 'component_container_isolate[d]' 'nav_benc[h]' 'parameter_bridg[e]' \
         'full_world_rob[o].launch.py' 'nav[.]launch.py' 'mission[.]launch.py'; do
    pgrep -af "$p" && { say "REFUSING: something is already running"; exit 3; }
done
if bash "$WT/gazebo_models/scripts/ros_clean.sh" --list | tail -n +2 | grep -q .; then
    bash "$WT/gazebo_models/scripts/ros_clean.sh" --list
    say "REFUSING: ros_clean.sh would kill something"; exit 3
fi

source "$WT/setup_env.sh" > /dev/null 2>&1
source "$WS/install/local_setup.bash" || exit 1
export PYTHONPATH="$WS/build/custom_teleop:$WS/build/coco_config:$PYTHONPATH"
# setup_env.sh computes the workspace as two directories above itself, which
# from a worktree is .claude/, and silently skips the MoveIt prefix (C2-NAV.40).
MV="/home/gautham/ros2_ws(personal)/moveit_prefix/root/opt/ros/jazzy"
PYVER="$(python3 -c 'import sys; print(f"python{sys.version_info.major}.{sys.version_info.minor}")')"
if [ -d "$MV" ]; then
    export AMENT_PREFIX_PATH="$MV:$AMENT_PREFIX_PATH"
    export CMAKE_PREFIX_PATH="$MV:$CMAKE_PREFIX_PATH"
    export LD_LIBRARY_PATH="$MV/lib:$MV/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH"
    export PYTHONPATH="$MV/lib/$PYVER/site-packages:$PYTHONPATH"
    export PATH="$MV/bin:$PATH"
fi
python3 -c 'import moveit_configs_utils' || { say "REFUSING: moveit_configs_utils not importable"; exit 5; }
{
    for pkg in coco_mission coco_perception coco_moveit_config coco_rl coco_web custom_teleop gazebo_models coco_config coco_sim; do
        printf '%s %s\n' "$pkg" "$(ros2 pkg prefix "$pkg" 2>&1)"
    done
    printf 'full_world_robo.launch.py %s\n' "$(ls "$(ros2 pkg prefix gazebo_models)/share/gazebo_models/launch/full_world_robo.launch.py" 2>&1)"
    printf 'mission.launch.py %s\n' "$(ls "$(ros2 pkg prefix coco_mission)/share/coco_mission/launch/mission.launch.py" 2>&1)"
    printf 'policy %s\n' "$(ls "$(ros2 pkg prefix coco_rl)/share/coco_rl/policies/phase5_24deg_s0p0.zip" 2>&1)"
} > "$OUT/resolve.txt"
cat "$OUT/resolve.txt"
for pkg in coco_mission coco_perception coco_moveit_config coco_rl coco_web custom_teleop gazebo_models coco_config coco_sim; do
    case "$(ros2 pkg prefix "$pkg" 2>/dev/null)" in
        "$WS"/install/*) ;;
        *) say "REFUSING: $pkg does not resolve into $WS/install"; exit 5;;
    esac
done
HEAD="$(git -C "$WT" rev-parse HEAD)"
DIRTY="$(git -C "$WT" status --porcelain | wc -l)"
say "git $HEAD, dirty paths: $DIRTY, colour $COLOUR"
{
    echo "head=$HEAD"
    echo "dirty_paths=$DIRTY"
    echo "colour=$COLOUR"
    echo "nav2_params_sha256=$(sha256sum "$WT/gazebo_models/config/nav2_params.yaml" | cut -d' ' -f1)"
    echo "started_utc=$(date -u +%Y-%m-%dT%H:%M:%SZ)"
} > "$OUT/meta.txt"

PGIDS=()
REC_PIDS=()
cleanup() {
    trap - EXIT INT TERM
    for pid in "${REC_PIDS[@]}"; do kill -INT "$pid" 2>/dev/null; done
    sleep 5
    for sig in INT TERM KILL; do
        alive=0
        for pg in "${PGIDS[@]}"; do kill -"$sig" -- "-$pg" 2>/dev/null && alive=1; done
        [ "$alive" = 1 ] || break
        sleep 8
    done
    bash "$WT/gazebo_models/scripts/ros_clean.sh" > "$OUT/ros_clean.log" 2>&1
    bash "$WT/gazebo_models/scripts/ros_clean.sh" --list > "$OUT/ros_clean_after.txt" 2>&1
    echo "ended_utc=$(date -u +%Y-%m-%dT%H:%M:%SZ)" >> "$OUT/meta.txt"
    if [ "$COMPLETED" = 1 ]; then
        say "torn down; runner checks $([ "$FAIL" = 0 ] && echo PASS || echo FAIL)"
    else
        say "torn down; INCOMPLETE -- stopped before every check ran, not a result"
    fi
}
trap cleanup EXIT
trap 'exit 130' INT TERM

wait_for() {
    local label="$1" budget="$2"; shift 2
    local deadline=$((SECONDS + budget))
    while [ "$SECONDS" -lt "$deadline" ]; do
        "$@" && { say "$label after $((SECONDS + budget - deadline))s"; return 0; }
        sleep 3
    done
    say "$label NOT reached in ${budget}s"; return 1
}
scan_up() { timeout 8 ros2 topic echo /scan --once --field header.stamp.sec > /dev/null 2>&1; }
NAV_NODES=(/bt_navigator /controller_server /planner_server /amcl /map_server
           /local_costmap/local_costmap /global_costmap/global_costmap
           /velocity_smoother /collision_monitor /behavior_server)
nav_active() {
    local n
    for n in "${NAV_NODES[@]}"; do
        timeout 8 ros2 lifecycle get "$n" 2>/dev/null | grep -q '^active' || return 1
    done
}

setsid ros2 launch gazebo_models full_world_robo.launch.py traverse:=true gui:=false \
    > "$OUT/sim.log" 2>&1 &
PGIDS+=("$!")
CHECK="sim publishing /scan"; check wait_for "$CHECK" 180 scan_up || exit 4

setsid ros2 launch coco_mission mission.launch.py rviz:=false target_colour:="$COLOUR" \
    > "$OUT/mission.log" 2>&1 &
PGIDS+=("$!")
CHECK="all Nav2 lifecycle nodes active"; check wait_for "$CHECK" 300 nav_active || exit 4
say "settling 60 s for move_group, perception and the web panel"
sleep 60

for n in "${NAV_NODES[@]}"; do
    printf '%s %s\n' "$n" "$(timeout 8 ros2 lifecycle get "$n" 2>&1)"
done > "$OUT/lifecycle.txt"
cat "$OUT/lifecycle.txt"

CHECK="live nav2 parameter readback matches shipped file"
check python3 -P "$WT/gazebo_models/scripts/nav_params_overlay.py" verify-live \
    --params "$WT/gazebo_models/config/nav2_params.yaml" --out "$OUT/params_live.txt"

ros2 node list > "$OUT/nodes.txt" 2>&1
for n in /mission_executive /localization_monitor /mission_hud /ramp_driver \
         /approach_server /grasp_server /move_group /bt_navigator /collision_monitor \
         /cmd_vel_relay /cmd_vel_arbiter /velocity_smoother; do
    CHECK="node $n present"; check grep -qx "$n" "$OUT/nodes.txt"
done
CHECK="exactly one node named like the arbiter"
check test "$(grep -c arbiter "$OUT/nodes.txt")" = 1

echo '{"topology": "B"}' > "$OUT/resolved_b.json"
python3 -P "$WT/gazebo_models/scripts/nav_params_overlay.py" verify-topology \
    --resolved "$OUT/resolved_b.json" --out "$OUT/topology_live.txt"
cat "$OUT/topology_live.txt"
CHECK="wheel owner + every command-chain link (arbiter-mode line excluded: idle is correct before start)"
check bash -c "! grep '^MISMATCH' '$OUT/topology_live.txt' | grep -v 'arbiter mode'"
CHECK="topology_live.txt contains the chain section"
check grep -q 'command chain, link by link' "$OUT/topology_live.txt"

for t in /cmd_vel_nav /cmd_vel_smoothed /cmd_vel /cmd_vel_gated /diff_drive_controller/cmd_vel \
         /cmd_vel_rl /cmd_vel_approach /cmd_vel_teleop; do
    echo "===== $t"
    ros2 topic info -v "$t"
done > "$OUT/graph_chain.txt" 2>&1

# depth fusion must be OFF: no depth-cloud process, both voxel layers read scan only
{
    echo "depth_cloud processes:"
    pgrep -af 'image_proc/resize_nod[e]|point_cloud_xyz_nod[e]|depth_cloud[.]launch' || echo "  none"
    for n in /local_costmap/local_costmap /global_costmap/global_costmap; do
        printf '%s voxel_layer.observation_sources=%s\n' "$n" \
            "$(timeout 10 ros2 param get "$n" voxel_layer.observation_sources 2>&1 | tr '\n' ' ')"
        printf '%s obstacle_layer.observation_sources=%s\n' "$n" \
            "$(timeout 10 ros2 param get "$n" obstacle_layer.observation_sources 2>&1 | tr '\n' ' ')"
    done
} > "$OUT/depth_off.txt"
cat "$OUT/depth_off.txt"
CHECK="depth fusion off (no depth-cloud process, every observation source is scan)"
check bash -c "grep -q '^  none$' '$OUT/depth_off.txt' && test \$(grep -c 'String value is: scan \$' '$OUT/depth_off.txt') = 4"

CHECK="no launched process died during bring-up"
check bash -c "! grep -E 'process has died|exited with code [1-9]' '$OUT/mission.log' '$OUT/sim.log'"

# --- the mission, nominal, nothing injected -------------------------------
python3 -P "$WT/docs/data/c2nav42_cmdpath.py" record --out "$OUT/cmdpath" \
    --duration "$MISSION_BUDGET" --until-terminal > "$OUT/cmdpath.log" 2>&1 &
REC_PIDS+=("$!")
CMDPATH_PID=$!
python3 -P "$WT/docs/data/c2m51_hrec.py" --out "$OUT/hrec.csv" --tag c2nav44_m6 \
    --stop-on-terminal > "$OUT/hrec.log" 2>&1 &
REC_PIDS+=("$!")
ros2 topic echo /mission/state --field data > "$OUT/state_stream.txt" 2>&1 &
REC_PIDS+=("$!")
sleep 8

say "starting the mission ($COLOUR)"
START_S=$SECONDS
timeout 30 ros2 service call /mission/start std_srvs/srv/Trigger > "$OUT/start.txt" 2>&1
cat "$OUT/start.txt"
CHECK="mission start accepted"; check grep -q 'success=True' "$OUT/start.txt"

terminal() {
    timeout 8 ros2 topic echo /mission/state --once --field data 2>/dev/null \
        | grep -Eq 'state=(COMPLETE|ABORT)\b'
}
wait_for "mission terminal" "$MISSION_BUDGET" terminal
say "wall seconds from start call to terminal detection: $((SECONDS - START_S))"
timeout 8 ros2 topic echo /mission/state --once --field data > "$OUT/final_state.txt" 2>&1
cat "$OUT/final_state.txt"
timeout 12 ros2 topic echo /amcl_pose --once > "$OUT/final_amcl_pose.txt" 2>&1
sleep 3
kill -INT "$CMDPATH_PID" 2>/dev/null
wait "$CMDPATH_PID"
tail -80 "$OUT/cmdpath.log"

CHECK="no launched process died during the mission"
check bash -c "! grep -E 'process has died|exited with code [1-9]' '$OUT/mission.log' '$OUT/sim.log'"
grep -E 'process has died|exited with code [1-9]' "$OUT/mission.log" "$OUT/sim.log" | head -10
COMPLETED=1
exit "$FAIL"
