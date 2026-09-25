#!/usr/bin/env bash
# episode_run.sh — execute one recorded episode in a fresh container.
#
#   scripts/container/episode_run.sh --image TAG --seed 7 --out DIR [--colour C]
#
#   1. The manifest is generated IN THE IMAGE from the seed (level fixed),
#      and -- when this host can import coco_sim from the source tree --
#      on the host too; the two must be byte-identical.
#   2. Only the manifest's task_view() crosses into the robot: its colour
#      becomes COCO_TARGET_COLOUR. No pose, no lane, no target list. The
#      privileged manifest stays with this script.
#   3. A fresh appliance container (hardened like compose, --network
#      none) runs the frozen mission: healthy, then LOCALISED (map->odom
#      exists -- /healthz 200 does not mean localised, CLAUDE.md), then
#      /mission/start, then COMPLETE or ABORT.
#   4. The run becomes an EpisodeResult (episode_tool.py record) and the
#      seed is regenerated to check the record still describes it.
#
# Only level `fixed` can be executed: it IS the frozen world's layout. The
# `colours` and `positions` levels need a world built from the manifest,
# which does not exist yet; this refuses them rather than run the wrong
# world.
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
IMAGE=""; SEED=""; OUT=""; COLOUR=""; LEVEL=fixed; BUDGET=2400
while [ $# -gt 0 ]; do
  case "$1" in
    --image) IMAGE=$2; shift ;;
    --seed) SEED=$2; shift ;;
    --out) OUT=$2; shift ;;
    --colour) COLOUR=$2; shift ;;
    --level) LEVEL=$2; shift ;;
    --budget) BUDGET=$2; shift ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
  shift
done
[ -n "$IMAGE" ] && [ -n "$SEED" ] && [ -n "$OUT" ] || { echo "--image, --seed, --out required" >&2; exit 2; }
[ "$LEVEL" = fixed ] || { echo "level $LEVEL cannot be executed: no manifest-built world exists yet" >&2; exit 4; }
read -r -a D <<< "${DOCKER:-docker}"
mkdir -p "$OUT/run" "$OUT/ros_log"; chmod -R 0777 "$OUT"
log() { echo "[episode] $*" | tee -a "$OUT/episode.log"; }
CARGS=(); [ -n "$COLOUR" ] && CARGS=(--colour "$COLOUR")

"${D[@]}" run --rm --network none -v "$HERE:/opt/harness:ro" "$IMAGE" \
  python3 /opt/harness/episode_tool.py generate --seed "$SEED" --level "$LEVEL" "${CARGS[@]}" \
  > "$OUT/manifest.json" || { log "manifest generation failed in the image"; exit 1; }
if PYTHONPATH="$REPO/coco_sim:$REPO/coco_config" python3 -c 'import coco_sim.episode' 2>/dev/null; then
  PYTHONPATH="$REPO/coco_sim:$REPO/coco_config" python3 "$HERE/episode_tool.py" generate \
    --seed "$SEED" --level "$LEVEL" "${CARGS[@]}" > "$OUT/manifest_host.json"
  if cmp -s "$OUT/manifest.json" "$OUT/manifest_host.json"; then
    log "manifest: host and image byte-identical ($(sha256sum < "$OUT/manifest.json" | cut -c1-16))"
  else
    log "manifest: host and image DIFFER"; exit 1
  fi
fi
"${D[@]}" run --rm --network none -v "$HERE:/opt/harness:ro" -v "$OUT:/out" "$IMAGE" \
  python3 /opt/harness/episode_tool.py task-view /out/manifest.json > "$OUT/task_view.json"
TARGET=$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["requested_colour"])' "$OUT/task_view.json")
log "task view: $(cat "$OUT/task_view.json")"

if pgrep -f 'g[z] sim' >/dev/null; then log "REFUSE: a gz sim is running on this host"; exit 3; fi
T0=$(date +%s)
CID=$("${D[@]}" run -d --network none --shm-size 2g --cap-drop ALL \
      --security-opt no-new-privileges:true --ulimit core=0 \
      -e COCO_TARGET_COLOUR="$TARGET" -e ROS_LOG_DIR=/out/ros_log \
      -v "$OUT:/out" "$IMAGE")
log "container $CID, target_colour=$TARGET"
ex() { "${D[@]}" exec "$CID" coco-entrypoint "$@"; }
state=""
until [ "$state" = healthy ] || [ $(( $(date +%s) - T0 )) -ge 900 ]; do
  sleep 5
  state=$("${D[@]}" inspect -f '{{.State.Health.Status}}' "$CID" 2>/dev/null)
  [ "$state" = unhealthy ] && break
done
log "health: $state after $(( $(date +%s) - T0 )) s"
LOCALISED=no
if [ "$state" = healthy ]; then
  for _ in $(seq 1 60); do
    # To a file, then grep: tf2_echo never exits on its own, so `timeout`
    # kills it (124), and under pipefail a pipe into grep would report
    # that failure precisely when the transform was found.
    ex timeout 10 ros2 run tf2_ros tf2_echo map odom > "$OUT/tf_map_odom.txt" 2>&1
    if grep Translation "$OUT/tf_map_odom.txt" >/dev/null; then
      LOCALISED=yes; break
    fi
    sleep 5
  done
fi
log "localised (map->odom): $LOCALISED after $(( $(date +%s) - T0 )) s"
if [ "$LOCALISED" = yes ]; then
  ex timeout 30 ros2 service call /mission/start std_srvs/srv/Trigger > "$OUT/start.txt" 2>&1
  log "start: $(grep -o 'success=[A-Za-z]*' "$OUT/start.txt")"
  TS=$(date +%s)
  while [ $(( $(date +%s) - TS )) -lt "$BUDGET" ]; do
    ex timeout 8 ros2 topic echo --once /mission/state --field data > "$OUT/run/final_state.txt" 2>/dev/null
    grep -E 'state=(COMPLETE|ABORT)\b' "$OUT/run/final_state.txt" >/dev/null && break
    sleep 5
  done
  log "terminal: $(head -1 "$OUT/run/final_state.txt" | cut -c1-160)"
fi
"${D[@]}" logs "$CID" > "$OUT/container.log" 2>&1
"${D[@]}" cp "$CID:/tmp/coco_sim.log" "$OUT/" >/dev/null 2>&1
"${D[@]}" cp "$CID:/tmp/coco_stack.log" "$OUT/" >/dev/null 2>&1
"${D[@]}" rm -f "$CID" >/dev/null
python3 "$HERE/extract_mission_result.py" "$OUT" "$TARGET" | tee -a "$OUT/episode.log"
COMMIT=$("${D[@]}" run --rm --network none --entrypoint cat "$IMAGE" /opt/coco/build-info.json \
  | python3 -c 'import json,sys; print(json.load(sys.stdin)["git_sha"])')
"${D[@]}" run --rm --network none -v "$HERE:/opt/harness:ro" -v "$OUT:/out" "$IMAGE" \
  python3 /opt/harness/episode_tool.py record /out/manifest.json /out/result.json "$COMMIT" \
  > "$OUT/episode_result.json" 2> "$OUT/record.err"
log "episode result: $(python3 -c 'import json,sys; r=json.load(open(sys.argv[1])); print(r["outcome"], r.get("failure_reason",""), "reproducible_from_seed=%s" % r["reproducible_from_seed"])' "$OUT/episode_result.json" 2>&1)"
