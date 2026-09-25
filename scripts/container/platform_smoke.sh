#!/usr/bin/env bash
# platform_smoke.sh — docs/DOCKER.md's verification procedure, executed and
# measured, against one image, through the real docker-compose.yml.
#
#   scripts/container/platform_smoke.sh --image TAG --out DIR [--timeout 900] [--project NAME]
#
# Refuses (exit 3) while any `gz sim` runs on this host: one Gazebo per
# machine, and a container's processes are visible to -- and, at the same
# uid, killable by -- host-side sweeps such as ros_clean.sh.
#
# Records: seconds from `up` to healthy, CPU/RAM every 5 s, /healthz,
# /api/session, /api/metrics, that 8081 is NOT reachable from the host,
# the listening sockets inside, the cmd_vel publisher set, a 30 s graph
# probe (topic rates, /clock RTF, TF), steady-state CPU/RAM, the logs, and
# the stop time, exit code and leftovers.
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
IMAGE=""; OUT=""; TMO=900; PROJ=cocosmoke; NAV=0; WEB=0
while [ $# -gt 0 ]; do
  case "$1" in
    --image) IMAGE=$2; shift ;;
    --out) OUT=$2; shift ;;
    --timeout) TMO=$2; shift ;;
    --project) PROJ=$2; shift ;;
    --nav) NAV=1 ;;   # also drive Nav2: nav_smoke.py (moves the robot)
    --web) WEB=1 ;;   # also speak coco.v1 from the host: ws_probe.py
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
  shift
done
[ -n "$IMAGE" ] && [ -n "$OUT" ] || { echo "--image and --out are required" >&2; exit 2; }
read -r -a D <<< "${DOCKER:-docker}"
CF="$REPO/docker-compose.yml"
export COCO_IMAGE="$IMAGE"
C=("${D[@]}" compose -p "$PROJ" -f "$CF")
mkdir -p "$OUT"
el() { awk -v a="$1" -v b="$(date +%s.%N)" 'BEGIN{printf "%.1f", b-a}'; }
R="$OUT/result.txt"; : > "$R"

if pgrep -af 'g[z] sim' > "$OUT/host_gz_before.txt"; then
  echo "REFUSE: a gz sim is already running on this host:" | tee -a "$R"
  cat "$OUT/host_gz_before.txt"; exit 3
fi
uptime > "$OUT/load_before.txt"
"${C[@]}" config > "$OUT/compose_resolved.yml" 2>&1
T0=$(date +%s.%N)
"${C[@]}" up -d --no-build > "$OUT/up.log" 2>&1
echo "up_rc=$? up_seconds=$(el "$T0")" | tee -a "$R"
CID=$("${C[@]}" ps -aq coco)
echo "cid=$CID" >> "$R"
echo "elapsed_s,health,running,cpu,mem" > "$OUT/stats.csv"
H=none; RUN=false; E=0
while :; do
  H=$("${D[@]}" inspect -f '{{if .State.Health}}{{.State.Health.Status}}{{else}}none{{end}}' "$CID")
  RUN=$("${D[@]}" inspect -f '{{.State.Running}}' "$CID")
  S=$("${D[@]}" stats --no-stream --format '{{.CPUPerc}},{{.MemUsage}}' "$CID" 2>/dev/null)
  E=$(el "$T0")
  echo "$E,$H,$RUN,$S" >> "$OUT/stats.csv"
  [ "$H" = healthy ] && break
  # Docker's own verdict after start_period + retries; waiting on is pointless.
  [ "$H" = unhealthy ] && break
  [ "$RUN" != true ] && break
  awk -v e="$E" -v t="$TMO" 'BEGIN{exit !(e>t)}' && break
  sleep 5
done
echo "final_health=$H running=$RUN seconds_to_state=$E" | tee -a "$R"
"${D[@]}" logs "$CID" > "$OUT/container_boot.log" 2>&1
for ep in healthz api/session api/metrics; do
  curl -s -m 5 -o "$OUT/$(basename "$ep").json" -w "${ep}_http=%{http_code}\n" \
    "http://127.0.0.1:${COCO_HTTP_PORT:-8080}/$ep" | tee -a "$R"
done
curl -s -m 5 -o /dev/null -w 'index_http=%{http_code}\n' "http://127.0.0.1:${COCO_HTTP_PORT:-8080}/" | tee -a "$R"
curl -s -m 3 -o /dev/null http://127.0.0.1:8081/
echo "host_8081_curl_exit=$? (7 = refused = PASS)" | tee -a "$R"
if [ "$RUN" = true ]; then
  for f in graph_probe.py ports.py; do "${D[@]}" cp "$HERE/$f" "$CID:/tmp/$f" >/dev/null; done
  "${D[@]}" exec "$CID" ss -ltn > "$OUT/listening.txt" 2>&1 \
    || "${D[@]}" exec "$CID" python3 /tmp/ports.py > "$OUT/listening.txt" 2>&1
  "${D[@]}" exec "$CID" id > "$OUT/id.txt" 2>&1
  "${D[@]}" exec "$CID" coco-entrypoint ros2 topic info /diff_drive_controller/cmd_vel -v > "$OUT/cmd_vel_info.txt" 2>&1
  "${D[@]}" exec "$CID" coco-entrypoint ros2 node list > "$OUT/nodes.txt" 2>&1
  "${D[@]}" exec "$CID" coco-entrypoint timeout 30 ros2 control list_controllers > "$OUT/controllers.txt" 2>&1
  "${D[@]}" exec "$CID" coco-entrypoint python3 /tmp/graph_probe.py 30 > "$OUT/graph_probe.json" 2> "$OUT/graph_probe.err"
  if [ "$WEB" = 1 ] && [ "$H" = healthy ]; then
    "${D[@]}" exec "$CID" coco-entrypoint ros2 topic list > "$OUT/topics.txt" 2>&1
    # From the HOST, through the published port, when the host has tornado;
    # otherwise from inside the container (said so in the output).
    if python3 -c 'import tornado' 2>/dev/null; then
      python3 "$HERE/ws_probe.py" "ws://127.0.0.1:${COCO_HTTP_PORT:-8080}/ws" "$OUT/topics.txt" 10 \
        > "$OUT/ws_probe.json" 2> "$OUT/ws_probe.err"; echo "ws_probe_from=host rc=$?" | tee -a "$R"
    else
      "${D[@]}" cp "$HERE/ws_probe.py" "$CID:/tmp/ws_probe.py" >/dev/null
      "${D[@]}" cp "$OUT/topics.txt" "$CID:/tmp/topics.txt" >/dev/null
      "${D[@]}" exec "$CID" python3 /tmp/ws_probe.py ws://127.0.0.1:8080/ws /tmp/topics.txt 10 \
        > "$OUT/ws_probe.json" 2> "$OUT/ws_probe.err"; echo "ws_probe_from=container rc=$?" | tee -a "$R"
    fi
  fi
  if [ "$NAV" = 1 ] && [ "$H" = healthy ]; then
    "${D[@]}" cp "$HERE/nav_smoke.py" "$CID:/tmp/nav_smoke.py" >/dev/null
    "${D[@]}" exec "$CID" coco-entrypoint python3 /tmp/nav_smoke.py > "$OUT/nav_smoke.json" 2> "$OUT/nav_smoke.err"
    echo "nav_smoke_rc=$?" | tee -a "$R"
  fi
  for _ in 1 2 3 4 5 6; do
    "${D[@]}" stats --no-stream --format '{{.CPUPerc}},{{.MemUsage}},{{.PIDs}}' "$CID" >> "$OUT/steady_stats.csv"
    sleep 5
  done
  # top, not ps: ps's %CPU is a lifetime average and flatters nothing.
  # The second of two 3 s samples is the instantaneous one.
  "${D[@]}" exec "$CID" top -b -n 2 -d 3 -o %CPU -w 200 > "$OUT/top_procs.txt" 2>&1
  for f in /tmp/coco_sim.log /tmp/coco_stack.log; do "${D[@]}" cp "$CID:$f" "$OUT/" >/dev/null 2>&1; done
fi
uptime > "$OUT/load_during.txt"
T1=$(date +%s.%N)
"${C[@]}" stop > "$OUT/stop.log" 2>&1
echo "stop_seconds=$(el "$T1") exit=$("${D[@]}" inspect -f '{{.State.ExitCode}} oom={{.State.OOMKilled}}' "$CID")" | tee -a "$R"
"${D[@]}" logs "$CID" > "$OUT/container_full.log" 2>&1
"${C[@]}" down > "$OUT/down.log" 2>&1
echo "leftover_containers=$("${D[@]}" ps -aq --filter "label=com.docker.compose.project=$PROJ" | wc -l)" | tee -a "$R"
pgrep -af 'g[z] sim' > "$OUT/host_gz_after.txt"
echo "host_gz_after=$(wc -l < "$OUT/host_gz_after.txt")" | tee -a "$R"
[ "$H" = healthy ]
