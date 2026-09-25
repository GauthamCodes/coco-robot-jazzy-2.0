#!/usr/bin/env bash
# mission_regression.sh — the colour-fetch regression, one fresh container
# per run, driven by the repo's own mission runner.
#
#   scripts/container/mission_regression.sh --image coco-platform:main \
#       --runner-ref main --colours "red green blue yellow" --out DIR [--reps 1]
#
# The runner is <runner-ref>:docs/data/navigation_world_run.sh -- the script
# the four-colour matrix on main was run with -- UNCHANGED. docs/data is not
# in the image (.dockerignore), so that directory is exported from the SAME
# ref with `git archive` and mounted read-only where the runner expects to
# live, <ws>/src/coco-robot-ros2/docs/data. The image must be built from
# that ref too (build.sh --ref REF --infra-ref <this branch>).
#
# What the container supplies that the host run had:
#   - a virtual display (xvfb-run): the runner forces rviz:=true
#   - COCO_TEST_GUI=false: server-only Gazebo, as three of the four
#     archived runs used
#   - ROS_LOG_DIR inside the mounted output, so the executive and grasp
#     logs -- where every result field is read from -- outlive the container
# and nothing else: --network none (the whole graph is inside), no
# capabilities, no-new-privileges, the image's non-root user.
#
# One run at a time, and never while any `gz sim` runs on this host
# (--wait-quiet S blocks until the host has been quiet for S seconds).
# A fresh container per run is not optional: the gz DetachableJoint binds
# its child once per simulator, so a second mission in the same simulator
# welds nothing and reports success.
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
IMAGE=""; RREF=""; COLOURS="red green blue yellow"; OUT=""; REPS=1; QUIET=0
# The runner's own worst case is 180 + 300 + 60 s of bring-up waits plus
# its 1800 s mission budget; this outer bound must never cut in first.
BUDGET=2700
while [ $# -gt 0 ]; do
  case "$1" in
    --image) IMAGE=$2; shift ;;
    --runner-ref) RREF=$2; shift ;;
    --colours) COLOURS=$2; shift ;;
    --out) OUT=$2; shift ;;
    --reps) REPS=$2; shift ;;
    --wait-quiet) QUIET=$2; shift ;;
    --budget) BUDGET=$2; shift ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
  shift
done
[ -n "$IMAGE" ] && [ -n "$RREF" ] && [ -n "$OUT" ] || {
  echo "--image, --runner-ref and --out are required" >&2; exit 2; }
read -r -a D <<< "${DOCKER:-docker}"
mkdir -p "$OUT"
RUNNER="$OUT/runner_src"
if [ ! -d "$RUNNER/docs/data" ]; then
  mkdir -p "$RUNNER"
  git -C "$REPO" archive --format=tar "$RREF" docs/data | tar -x -C "$RUNNER"
fi
git -C "$REPO" rev-parse --verify "$RREF^{commit}" > "$OUT/runner_ref_sha.txt"
"${D[@]}" run --rm --network none "$IMAGE" info > "$OUT/image_info.json" 2>&1

host_busy() { pgrep -f 'g[z] sim' >/dev/null; }
# Seconds the host has spent suspended since boot: CLOCK_BOOTTIME advances
# through a suspend, CLOCK_MONOTONIC does not. Measured: a lid closed for
# 21 min in the middle of a mission froze the simulator while the runner's
# wall-clock budget kept counting, and the run "timed out" with the robot
# already home. A run that spans a suspend is VOID, and says so.
slept_s() {
  python3 -c 'import time; print(round(time.clock_gettime(time.CLOCK_BOOTTIME) - time.clock_gettime(time.CLOCK_MONOTONIC), 1))'
}
free_gb() { df -BG --output=avail "$OUT" | tail -1 | tr -dc '0-9'; }
CURRENT=""
cleanup() { [ -n "$CURRENT" ] && "${D[@]}" rm -f "$CURRENT" >/dev/null 2>&1; }
trap cleanup EXIT INT TERM
wait_quiet() {
  local since; since=$(date +%s)
  while :; do
    if host_busy; then since=$(date +%s)
    elif [ $(( $(date +%s) - since )) -ge "$QUIET" ]; then return 0; fi
    sleep 10
  done
}

for rep in $(seq 1 "$REPS"); do
  for colour in $COLOURS; do
    RUN="$OUT/$colour-rep$rep"
    mkdir -p "$RUN/run" "$RUN/ros_log"; chmod -R 0777 "$RUN"
    [ "$QUIET" -gt 0 ] && wait_quiet
    if host_busy; then
      echo "[regression] REFUSE $colour rep $rep: a gz sim is running on this host" | tee "$RUN/refused.txt"
      pgrep -af 'g[z] sim' >> "$RUN/refused.txt"; continue
    fi
    # Measured: a full disk voided a run (the stack could not start).
    if [ "$(free_gb)" -lt 2 ]; then
      echo "[regression] REFUSE $colour rep $rep: under 2 GB free" | tee "$RUN/refused.txt"; continue
    fi
    NAME="coco-mission-$colour-$rep-$$"
    echo "[regression] $colour rep $rep -> $RUN"
    uptime > "$RUN/load_before.txt"
    SLEPT0=$(slept_s)
    T0=$(date +%s.%N)
    # Detached and named, not `docker run --rm` in the foreground: measured,
    # when the foreground client died (writing its log to a full disk) the
    # container kept running with nobody to stop it. Now the container's
    # lifetime is this script's to end, whatever happens to a client.
    CURRENT=$("${D[@]}" run -d --name "$NAME" --network none \
      --security-opt no-new-privileges:true --cap-drop ALL --shm-size 2g \
      --ulimit core=0 \
      -e COCO_TEST_GUI=false -e ROS_LOG_DIR=/out/ros_log \
      -v "$RUNNER/docs/data:/opt/coco_ws/src/coco-robot-ros2/docs/data:ro" \
      -v "$RUN:/out" \
      "$IMAGE" xvfb-run -a -s "-screen 0 1280x1024x24" \
      bash /opt/coco_ws/src/coco-robot-ros2/docs/data/navigation_world_run.sh /out/run "$colour")
    "${D[@]}" logs -f "$CURRENT" > "$RUN/container.log" 2>&1 &
    LPID=$!
    echo "t_s,cpu,mem,pids" > "$RUN/stats.csv"
    RC=timeout
    while [ "$(awk -v a="$T0" -v b="$(date +%s.%N)" 'BEGIN{print int(b-a)}')" -lt "$BUDGET" ]; do
      if [ "$("${D[@]}" inspect -f '{{.State.Running}}' "$CURRENT" 2>/dev/null)" != true ]; then
        RC=$("${D[@]}" inspect -f '{{.State.ExitCode}}' "$CURRENT"); break
      fi
      S=$("${D[@]}" stats --no-stream --format '{{.CPUPerc}},{{.MemUsage}},{{.PIDs}}' "$CURRENT" 2>/dev/null)
      [ -n "$S" ] && echo "$(awk -v a="$T0" -v b="$(date +%s.%N)" 'BEGIN{printf "%.0f", b-a}'),$S" >> "$RUN/stats.csv"
      sleep 10
    done
    "${D[@]}" rm -f "$CURRENT" >/dev/null 2>&1; CURRENT=""
    kill "$LPID" 2>/dev/null
    T1=$(date +%s.%N)
    SLEPT=$(awk -v a="$SLEPT0" -v b="$(slept_s)" 'BEGIN{printf "%.1f", b-a}')
    uptime > "$RUN/load_after.txt"
    VOID=""
    awk -v s="$SLEPT" 'BEGIN{exit !(s > 5)}' && VOID="host suspended ${SLEPT} s during the run"
    [ -n "$VOID" ] && echo "$VOID" > "$RUN/VOID.txt"
    echo "container_rc=$RC container_wall_s=$(awk -v a="$T0" -v b="$T1" 'BEGIN{printf "%.1f", b-a}') host_slept_s=$SLEPT ${VOID:+VOID}" | tee "$RUN/container_result.txt"
    python3 "$HERE/extract_mission_result.py" "$RUN" "$colour" | tee -a "$RUN/container_result.txt"
  done
done
python3 - "$OUT" <<'EOF'
import glob, json, os, sys
out = sys.argv[1]
rows = []
for f in sorted(glob.glob(f'{out}/*-rep*/result.json')):
    r = json.load(open(f))
    r.pop('transitions', None)
    d = os.path.dirname(f)
    r['run'] = os.path.basename(d)
    r['void'] = open(f'{d}/VOID.txt').read().strip() \
        if os.path.exists(f'{d}/VOID.txt') else None
    rows.append(r)
json.dump(rows, open(f'{out}/regression.json', 'w'), indent=1)
for r in rows:
    print(r['run'], r['outcome'], r['wall_duration_s'], r['localization_recoveries'],
          r['home_arrival_error_m'], r['runner_checks_failed'])
EOF
