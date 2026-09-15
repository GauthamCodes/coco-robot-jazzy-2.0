#!/usr/bin/env bash
# Copyright 2026 Gautham Anil
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# nav_tour_run.sh -- ONE fresh simulator, ONE bounded nav_bench tour, ONE run
# directory. Replaces the untracked .navbench/c2nN_run.sh helpers.
#
#   gazebo_models/scripts/nav_tour_run.sh gazebo_models/config/experiments/baseline.yaml
#
# In order:
#   1. refuses to start while any simulator, Nav2 container or nav_bench is
#      already running -- one Gazebo at a time, and never tearing down a run
#      someone else started
#   2. uses this worktree's overlay and checks the parameter file nav2 will
#      load resolves into this worktree
#   3. allocates ~/coco_nav_runs/<experiment>/<experiment>_rNN (next free NN,
#      never overwritten) and resolves the experiment file into it
#   4. headless sim -> /scan -> nav.launch.py -> every lifecycle node active
#   5. reads the accepted and safety parameters back off the live nodes and
#      stops if any differs from the file
#   6. optionally swaps in the instrumented AMCL and starts the GT sidecar
#   7. runs nav_bench.py, bounded by a wall-clock budget
#   8. unloads the instrumented AMCL (its destructor writes the capture
#      footer), stops the sidecar, validates and joins the capture
#   9. tears down its own process groups, then sweeps with ros_clean.sh
#
# Outputs in the run directory: manifest.json, experiment_resolved.json,
# params_live.txt, sim.log, nav.log, nav_bench.log, <run>.json and
# <run>_traces/, and with the diagnostic enabled diag.jsonl, gt.csv,
# gt.csv.meta.json, joined.csv, diag_summary.json.
#
# Naming: nothing this script puts on a command line contains ros_clean.sh's
# 'nav[2]_' pattern -- nav.launch.py loads its default parameter file, and a
# merged file is called params_merged.yaml.

set -o pipefail

EXP_IN="${1:-}"
[ -n "$EXP_IN" ] && [ -f "$EXP_IN" ] || { echo "usage: nav_tour_run.sh <experiment.yaml>"; exit 2; }
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WT="$(cd "$HERE/../.." && pwd)"
EXP="$(readlink -f "$EXP_IN")"
RUNS_ROOT="${COCO_NAV_RUNS:-$HOME/coco_nav_runs}"

die() { echo "nav_tour_run: FAILED: $*" >&2; exit "${2:-1}"; }
say() { echo "nav_tour_run: $* ($(date -u +%H:%M:%S) UTC)"; }

# --- 1. one Gazebo at a time -------------------------------------------
BUSY_PATTERNS=('g[z] sim' 'component_container_isolate[d]' 'nav_benc[h]'
               'parameter_bridg[e]' 'full_world_rob[o].launch.py'
               'nav[.]launch.py' 'mission[.]launch.py')
busy() { for p in "${BUSY_PATTERNS[@]}"; do pgrep -af "$p"; done; }
if [ -n "$(busy)" ]; then
    echo "nav_tour_run: REFUSING to start, a simulator or nav stack is already running:" >&2
    busy >&2
    exit 3
fi
# ros_clean.sh kills by command-line SUBSTRING. Nothing else is running at
# this point, so anything it would kill now is this runner's own process
# tree -- which the teardown sweep would then kill mid-cleanup (measured
# once, C2-NAV.39) -- or an unrelated process the sweep would take anyway.
if bash "$HERE/ros_clean.sh" --list | tail -n +2 | grep -q .; then
    echo "nav_tour_run: REFUSING: ros_clean.sh would kill these (possibly this runner):" >&2
    bash "$HERE/ros_clean.sh" --list >&2
    exit 3
fi

# --- 2. environment ------------------------------------------------------
# setup_env.sh sets RMW/Cyclone/GZ variables; the overlay is this worktree's.
source "$WT/setup_env.sh" 2>/dev/null
source "$WT/install/local_setup.bash" || die "no overlay at $WT/install; build first"
# ament_python editable installs keep their metadata under build/ (C2-NAV.37).
export PYTHONPATH="$WT/build/custom_teleop:$WT/build/coco_config:$PYTHONPATH"
python3 -c "import importlib.metadata as m; m.distribution('custom-teleop')" 2>/dev/null \
    || die "custom_teleop package metadata not importable (cmd_vel_relay would crash)"

GZ_PREFIX="$(ros2 pkg prefix gazebo_models)" || die "gazebo_models not found"
case "$GZ_PREFIX" in "$WT"/install/*) ;; *) die "gazebo_models resolves to $GZ_PREFIX, not $WT";; esac
BASE_PARAMS="$(readlink -f "$GZ_PREFIX/share/gazebo_models/config/nav2_params.yaml")"
[ "$BASE_PARAMS" = "$WT/gazebo_models/config/nav2_params.yaml" ] \
    || die "installed parameter file resolves to $BASE_PARAMS"

# --- 3. run directory ----------------------------------------------------
NAME="$(python3 -c 'import sys, yaml; print(yaml.safe_load(open(sys.argv[1]))["name"])' "$EXP")" \
    || die "cannot read experiment name from $EXP"
mkdir -p "$RUNS_ROOT/$NAME"
N=1
while [ -e "$RUNS_ROOT/$NAME/$(printf '%s_r%02d' "$NAME" "$N")" ]; do N=$((N + 1)); done
RUN_ID="$(printf '%s_r%02d' "$NAME" "$N")"
RUN="$RUNS_ROOT/$NAME/$RUN_ID"
mkdir "$RUN" || die "cannot create $RUN"
say "run $RUN_ID in $RUN"

python3 -P "$HERE/nav_params_overlay.py" resolve "$EXP" --base "$BASE_PARAMS" --out-dir "$RUN" \
    | tee "$RUN/overlay.log" || die "experiment refused (see $RUN/overlay.log)"
RESOLVED="$RUN/experiment_resolved.json"
jget() {
    python3 -c '
import json, sys
v = json.load(open(sys.argv[1]))
for k in sys.argv[2:]:
    v = v[k]
print("" if v is None else (str(v).lower() if isinstance(v, bool) else v))' "$RESOLVED" "$@"
}
PARAMS_FILE="$(jget params_file)"
REPEATS="$(jget bench repeats)"
TIMEOUT="$(jget bench timeout)"
ONLY="$(jget bench only)"
DIAG="$(jget amcl_diag enabled)"
# C2-NAV.40: bench.goals, one nav_bench --goal NAME:X,Y per moved scenario.
mapfile -t GOAL_ARGS < <(python3 -c '
import json, sys
for g in json.load(open(sys.argv[1]))["goal_args"]:
    print(g)' "$RESOLVED") || die "cannot read goal_args from $RESOLVED"

manifest() {
    python3 - "$RUN/manifest.json" "$@" <<'PY'
import json, os, sys
path, pairs = sys.argv[1], sys.argv[2:]
doc = json.load(open(path)) if os.path.exists(path) else {}
for pair in pairs:
    key, _, raw = pair.partition('=')
    try:
        doc[key] = json.loads(raw)
    except json.JSONDecodeError:
        doc[key] = raw
json.dump(doc, open(path, 'w'), indent=1)
PY
}
manifest "run_id=$RUN_ID" "experiment=$NAME" "experiment_file=$EXP" \
    "worktree=$WT" "git_sha=$(git -C "$WT" rev-parse HEAD)" \
    "git_dirty_paths=$(git -C "$WT" status --porcelain --untracked-files=no | wc -l)" \
    "params_file=$PARAMS_FILE" "params_sha256=$(sha256sum "$PARAMS_FILE" | cut -d' ' -f1)" \
    "started_utc=$(date -u +%FT%TZ)" "argv=$*"

# --- teardown ------------------------------------------------------------
PGIDS=()
SIDECAR_PID=""
DIAG_LOADED=0
stop_diag() {
    if [ "$DIAG_LOADED" = 1 ]; then
        DIAG_LOADED=0
        say "unloading instrumented AMCL (writes the capture footer)"
        timeout 90 python3 -P "$WT/coco_nav_diag/scripts/amcl_diag_swap.py" unload \
            >> "$RUN/amcl_diag_swap.log" 2>&1
    fi
    if [ -n "$SIDECAR_PID" ]; then
        kill -INT "$SIDECAR_PID" 2>/dev/null
        for _ in $(seq 1 20); do kill -0 "$SIDECAR_PID" 2>/dev/null || break; sleep 0.5; done
        SIDECAR_PID=""
    fi
}
cleanup() {
    local rc=$?
    trap - EXIT INT TERM
    stop_diag
    for sig in INT TERM KILL; do
        alive=0
        for pg in "${PGIDS[@]}"; do
            kill -"$sig" -- "-$pg" 2>/dev/null && alive=1
        done
        [ "$alive" = 1 ] || break
        sleep 8
    done
    bash "$HERE/ros_clean.sh" > "$RUN/ros_clean.log" 2>&1
    manifest "finished_utc=$(date -u +%FT%TZ)" "exit_code=$rc"
    say "done rc=$rc, run directory $RUN"
    exit "$rc"
}
trap cleanup EXIT
trap 'exit 130' INT TERM

wait_for() {  # wait_for <label> <seconds> <pid-that-must-stay-alive> <command...>
    local label="$1" budget="$2" pid="$3"
    shift 3
    local deadline=$((SECONDS + budget))
    while [ "$SECONDS" -lt "$deadline" ]; do
        kill -0 "$pid" 2>/dev/null || die "$label: launch process exited early" 4
        "$@" && { say "$label after $((SECONDS + budget - deadline))s"; return 0; }
        sleep 2
    done
    die "$label: not reached within ${budget}s" 4
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

# --- 4. bring-up -----------------------------------------------------------
setsid ros2 launch gazebo_models full_world_robo.launch.py gui:=false > "$RUN/sim.log" 2>&1 &
SIM_PID=$!
PGIDS+=("$SIM_PID")
wait_for "sim publishing /scan" 180 "$SIM_PID" scan_up

NAV_ARGS=(arbiter:=false)
[ "$PARAMS_FILE" != "$BASE_PARAMS" ] && NAV_ARGS+=("params_file:=$PARAMS_FILE")
setsid ros2 launch gazebo_models nav.launch.py "${NAV_ARGS[@]}" > "$RUN/nav.log" 2>&1 &
NAV_PID=$!
PGIDS+=("$NAV_PID")
wait_for "all Nav2 lifecycle nodes active" 240 "$NAV_PID" nav_active

# --- 5. the parameters that actually loaded -------------------------------
python3 -P "$HERE/nav_params_overlay.py" verify-live --params "$PARAMS_FILE" \
    --out "$RUN/params_live.txt" || die "live parameters differ from $PARAMS_FILE" 5

# --- 6. optional AMCL odometry-input capture --------------------------------
if [ "$DIAG" = "true" ]; then
    python3 -P "$WT/docs/data/c2nav36_gt_sidecar.py" --out "$RUN/gt.csv" \
        > "$RUN/gt_sidecar.log" 2>&1 &
    SIDECAR_PID=$!
    DIAG_LOADED=1
    python3 -P "$WT/coco_nav_diag/scripts/amcl_diag_swap.py" load --params "$PARAMS_FILE" \
        --diag-output "$RUN/diag.jsonl" > "$RUN/amcl_diag_swap.log" 2>&1 \
        || die "AMCL diagnostic swap failed (see $RUN/amcl_diag_swap.log)" 6
    wait_for "instrumented /amcl active" 60 "$NAV_PID" \
        bash -c 'timeout 8 ros2 lifecycle get /amcl 2>/dev/null | grep -q "^active"'
fi

# --- 7. the tour -------------------------------------------------------------
BENCH=(--tag "$RUN_ID" --repeats "$REPEATS" --timeout "$TIMEOUT" --out "$RUN")
[ -n "$ONLY" ] && BENCH+=(--only "$ONLY")
for g in "${GOAL_ARGS[@]}"; do BENCH+=(--goal "$g"); done
LEGS=7
[ -n "$ONLY" ] && LEGS=$(echo "$ONLY" | tr ',' '\n' | wc -l)
BUDGET=$(python3 -c "print(int($LEGS * $REPEATS * ($TIMEOUT + 15) + 300))")
say "nav_bench ${BENCH[*]} (budget ${BUDGET}s)"
timeout "$BUDGET" python3 "$HERE/nav_bench.py" "${BENCH[@]}" > "$RUN/nav_bench.log" 2>&1
BENCH_RC=$?
manifest "nav_bench_exit_code=$BENCH_RC"
if [ "$BENCH_RC" = 139 ] && grep -qF "[nav_bench] wrote $RUN/$RUN_ID.json" "$RUN/nav_bench.log"; then
    # Pre-existing: nav_bench.py intermittently segfaults in rclpy teardown
    # AFTER writing its results (9 of 80 historical .navbench runs). The tour
    # data is complete; the crash is recorded in the manifest, not hidden.
    say "nav_bench crashed at shutdown after writing its results (exit 139)"
    manifest "nav_bench_shutdown_crash=true"
    BENCH_RC=0
fi
grep -E '^\[nav_bench\]   (SUCCEEDED|TIMEOUT|FAILED|ABORTED|CANCELED|FAILURE_CONTEXT)' \
    "$RUN/nav_bench.log"
# The goal each leg recorded against the goal the experiment asked for. A
# mismatch means the tour did not test the experiment, so the run fails.
if [ -f "$RUN/$RUN_ID.json" ]; then
    python3 -P "$HERE/nav_params_overlay.py" verify-goals --resolved "$RESOLVED" \
        --bench "$RUN/$RUN_ID.json" --out "$RUN/goals_check.txt"
    GOALS_RC=$?
    manifest "goals_check_exit_code=$GOALS_RC"
    [ "$GOALS_RC" = 0 ] || { say "driven goals differ from the experiment (see goals_check.txt)"; BENCH_RC=7; }
fi

# --- 8. close and validate the capture ---------------------------------------
if [ "$DIAG" = "true" ]; then
    stop_diag
    python3 -P "$WT/docs/data/c2nav36_diag.py" join "$RUN/diag.jsonl" "$RUN/gt.csv" \
        --out "$RUN/joined.csv" --summary-json "$RUN/diag_summary.json" \
        > "$RUN/diag_join.log" 2>&1
    DIAG_RC=$?
    manifest "diag_join_exit_code=$DIAG_RC"
    head -12 "$RUN/diag_join.log"
fi

python3 - "$RUN/$RUN_ID.json" <<'PY' | tee "$RUN/outcome.txt"
import collections, json, sys
legs = json.load(open(sys.argv[1]))['legs']
count = collections.Counter(l['status'] for l in legs)
print(f"legs {len(legs)}: " + ', '.join(f'{k} {v}' for k, v in sorted(count.items())))
PY
exit "$BENCH_RC"
