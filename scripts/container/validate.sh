#!/usr/bin/env bash
# validate.sh — prove a COCO image, stage by stage, and keep the evidence.
#
#   scripts/container/validate.sh --image coco-platform:<sha12>
#   scripts/container/validate.sh --image TAG --test-reps 3 --jobs 1
#   scripts/container/validate.sh --image TAG --sim --sim-reps 3
#
# Stages (each PASS/FAIL in validation.json; a FAIL never stops the rest):
#   preflight     docker/compose/buildx versions, host facts
#   identity      image id, size, layers, `coco-entrypoint info`
#   environment   env inside the container; host variables must NOT leak in
#                 (this script sets decoys and checks they are absent), the
#                 package path is the image's own, the user is not root
#   dependencies  rosdep check (exec + test) satisfied; pip layer == lock
#   tests         every package's suite, N times, --network none, no caps
#   platform      (--sim) docs/DOCKER.md's procedure: compose up, healthy,
#                 /healthz, 8081 not reachable, one cmd_vel publisher, the
#                 ROS graph measured, clean shutdown. Refuses to start while
#                 ANY gz sim runs on this host (one Gazebo at a time).
#
# A failed stage leaves a reproduction bundle in OUT/failures/<stage>/:
# commit, image id, the exact command, a filtered environment, ROS distro/
# RMW/domain, the image's dpkg + pip manifests, container and ROS logs, and
# timing. Values of variables named like a secret are redacted.
#
# DOCKER overrides the client, as in build.sh.
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
IMAGE=""; OUT=""; TEST_REPS=1; JOBS=1; SIM=0; SIM_REPS=1; HEALTH_TIMEOUT=900
SMOKE_FLAGS=()
while [ $# -gt 0 ]; do
  case "$1" in
    --nav|--web) SMOKE_FLAGS+=("$1") ;;   # passed to platform_smoke.sh
    --image) IMAGE=$2; shift ;;
    --out) OUT=$2; shift ;;
    --test-reps) TEST_REPS=$2; shift ;;
    --jobs) JOBS=$2; shift ;;
    --sim) SIM=1 ;;
    --sim-reps) SIM_REPS=$2; shift ;;
    --health-timeout) HEALTH_TIMEOUT=$2; shift ;;
    -h|--help) sed -n '2,30p' "$0"; exit 0 ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
  shift
done
[ -n "$IMAGE" ] || { echo "--image is required" >&2; exit 2; }
read -r -a D <<< "${DOCKER:-docker}"
STAMP=$(date -u +%Y%m%dT%H%M%SZ)
OUT=${OUT:-${COCO_EVIDENCE_DIR:-$HOME/coco_container_evidence}/validate/$(echo "$IMAGE" | tr '/:' '__')-$STAMP}
mkdir -p "$OUT"
STAGES="$OUT/stages.jsonl"; : > "$STAGES"
# Every container this script starts runs with the appliance's privileges.
# core=0: see docker-compose.yml (host apport + Docker's unlimited default
# filled this machine's disk with 12 GB of cores once).
HARDEN=(--security-opt no-new-privileges:true --cap-drop ALL --shm-size 2g --ulimit core=0)

now() { date +%s.%N; }
secs() { awk -v a="$1" -v b="$2" 'BEGIN{printf "%.1f", b-a}'; }
log() { echo "[validate] $*"; }

# ---- failure bundle ---------------------------------------------------------
capture_failure() {  # stage rc cmd [container]
  local stage=$1 rc=$2 cmd=$3 cid=${4:-} dir="$OUT/failures/$1"
  mkdir -p "$dir"
  "${D[@]}" run --rm --network none --entrypoint cat "$IMAGE" /opt/coco/build-info.json \
    > "$dir/build-info.json" 2>&1
  # The image's own manifests; an image built without them (the original
  # Dockerfile) is asked directly, so a bundle always has versions.
  "${D[@]}" run --rm --network none --entrypoint cat "$IMAGE" /opt/coco/manifest/dpkg.txt \
    > "$dir/dpkg.txt" 2>/dev/null \
    || "${D[@]}" run --rm --network none --entrypoint dpkg-query "$IMAGE" \
         -W -f='${Package}=${Version}\n' > "$dir/dpkg.txt" 2>&1
  "${D[@]}" run --rm --network none --entrypoint cat "$IMAGE" /opt/coco/manifest/pip.txt \
    > "$dir/pip.txt" 2>/dev/null \
    || "${D[@]}" run --rm --network none --entrypoint pip3 "$IMAGE" \
         list --format=freeze > "$dir/pip.txt" 2>&1
  "${D[@]}" run --rm --network none "$IMAGE" env 2>/dev/null | sort \
    | grep -E '^(ROS_|RMW_|CYCLONEDDS|GZ_|COCO_|AMENT_|COLCON_|PYTHONPATH|LD_LIBRARY_PATH|PATH|HOME|USER|LIBGL|DISPLAY)' \
    | sed -E 's/^([^=]*(TOKEN|SECRET|PASSWORD|PASSWD|CREDENTIAL|_KEY)[^=]*)=.*/\1=<redacted>/' \
    > "$dir/env.txt"
  if [ -n "$cid" ]; then
    "${D[@]}" logs "$cid" > "$dir/container.log" 2>&1
    for f in /tmp/coco_sim.log /tmp/coco_stack.log; do
      "${D[@]}" cp "$cid:$f" "$dir/" >/dev/null 2>&1
    done
    "${D[@]}" cp "$cid:/home/coco/.ros/log" "$dir/ros_log" >/dev/null 2>&1
  fi
  python3 - "$dir" "$stage" "$rc" "$cmd" "$IMAGE" "$(printf '%s ' "${D[@]}")" <<'EOF'
import json, os, platform, subprocess, sys, time
d, stage, rc, cmd, image, docker = sys.argv[1:7]
def sh(c):
    try:
        return subprocess.run(c, shell=True, capture_output=True, text=True,
                              timeout=60).stdout.strip()
    except Exception as e:  # noqa: BLE001
        return f'error: {e}'
bi = {}
try:
    bi = json.load(open(f'{d}/build-info.json'))
except Exception:
    pass
env = dict(l.split('=', 1) for l in open(f'{d}/env.txt') if '=' in l)
ctx = {
    'stage': stage, 'rc': rc, 'command': cmd,
    'captured_utc': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
    'image': image,
    'image_id': sh(f"{docker} image inspect -f '{{{{.Id}}}}' {image}"),
    'repo_digests': sh(f"{docker} image inspect -f '{{{{json .RepoDigests}}}}' {image}"),
    'git_sha': bi.get('git_sha'), 'infra_sha': bi.get('infra_sha'),
    'base_image': bi.get('base_image'),
    'ros_distro': env.get('ROS_DISTRO', '').strip(),
    'rmw': env.get('RMW_IMPLEMENTATION', '').strip(),
    'ros_domain_id': env.get('ROS_DOMAIN_ID', 'unset (0)').strip(),
    'host': {'kernel': platform.release(), 'cpus': os.cpu_count(),
             'mem_total_kb': sh("awk '/MemTotal/{print $2}' /proc/meminfo"),
             'load': open('/proc/loadavg').read().strip()},
    'docker_version': sh(f"{docker} version --format '{{{{.Server.Version}}}}'"),
}
json.dump(ctx, open(f'{d}/context.json', 'w'), indent=1)
EOF
  log "failure bundle: $dir"
}

run_stage() {  # name function
  local name=$1 fn=$2 t0 t1 rc
  log "== $name"
  t0=$(now); "$fn"; rc=$?; t1=$(now)
  printf '{"stage":"%s","rc":%d,"seconds":%s}\n' "$name" "$rc" "$(secs "$t0" "$t1")" >> "$STAGES"
  log "   $name -> $([ "$rc" = 0 ] && echo PASS || echo "FAIL (rc=$rc)")"
  return 0
}

# ---- stages -------------------------------------------------------------------
st_preflight() {
  local d="$OUT/preflight"; mkdir -p "$d"
  { "${D[@]}" version; echo; "${D[@]}" compose version; "${D[@]}" buildx version; } > "$d/docker.txt" 2>&1 || return 1
  "${D[@]}" info --format '{{json .}}' > "$d/docker_info.json" 2>/dev/null
  { uname -a; nproc; grep -E 'MemTotal|MemAvailable' /proc/meminfo; df -h / | tail -1
    command -v nvidia-smi >/dev/null && nvidia-smi -L
    echo "nvidia runtime: $("${D[@]}" info --format '{{json .Runtimes}}' | grep -c nvidia)"
  } > "$d/host.txt" 2>&1
  return 0
}

st_identity() {
  local d="$OUT/identity"; mkdir -p "$d"
  "${D[@]}" image inspect "$IMAGE" > "$d/image_inspect.json" || { capture_failure identity 1 "image inspect"; return 1; }
  "${D[@]}" history --no-trunc --format '{{.Size}}\t{{.CreatedBy}}' "$IMAGE" > "$d/layers.tsv"
  "${D[@]}" run --rm --network none "$IMAGE" info > "$d/info.json" 2>"$d/info.err" \
    || { capture_failure identity 1 "coco-entrypoint info"; return 1; }
  python3 -c "import json,sys; json.load(open(sys.argv[1]))" "$d/info.json" || return 1
}

st_environment() {
  local d="$OUT/environment"; mkdir -p "$d"
  # Decoys: if the container inherited the caller's environment, these
  # would appear inside. docker run passes nothing it is not told to.
  ROS_DOMAIN_ID=97 AMENT_PREFIX_PATH=/decoy/host/install COLCON_PREFIX_PATH=/decoy/host \
  ROS_DISTRO=humble PYTHONPATH=/decoy/pythonpath GZ_SIM_RESOURCE_PATH=/decoy/gz \
    "${D[@]}" run --rm --network none "$IMAGE" env > "$d/env_sourced.txt" 2>&1 || return 1
  sort -o "$d/env_sourced.txt" "$d/env_sourced.txt"
  "${D[@]}" run --rm --network none --entrypoint env "$IMAGE" | sort > "$d/env_config.txt"
  "${D[@]}" run --rm --network none "$IMAGE" id > "$d/id.txt"
  python3 - "$d" <<'EOF' || { capture_failure environment 1 "env checks"; return 1; }
import json, sys
d = sys.argv[1]
env = dict(l.rstrip('\n').split('=', 1) for l in open(f'{d}/env_sourced.txt') if '=' in l)
checks = {}
checks['no_decoy_leaked'] = not any('/decoy/' in v for v in env.values()) \
    and env.get('ROS_DISTRO') == 'jazzy' and env.get('ROS_DOMAIN_ID') is None
checks['no_host_home_path'] = not any('/home/' in v.replace('/home/coco', '')
                                      for v in env.values())
ament = [p for p in env.get('AMENT_PREFIX_PATH', '').split(':') if p]
checks['ament_prefix_is_image_own'] = bool(ament) and all(
    p == '/opt/ros/jazzy' or p.startswith('/opt/coco_ws/install/') for p in ament)
checks['coco_packages_on_path'] = sum(p.startswith('/opt/coco_ws/install/') for p in ament) == 9
checks['rmw_cyclonedds'] = env.get('RMW_IMPLEMENTATION') == 'rmw_cyclonedds_cpp'
idline = open(f'{d}/id.txt').read()
checks['not_root'] = 'uid=0(' not in idline
checks['uid'] = idline.split()[0] if idline else ''
json.dump(checks, open(f'{d}/checks.json', 'w'), indent=1)
print(json.dumps(checks))
sys.exit(0 if all(v for k, v in checks.items() if k != 'uid') else 1)
EOF
}

st_dependencies() {
  local d="$OUT/dependencies" rc=0
  mkdir -p "$d"
  for t in exec test; do
    "${D[@]}" run --rm --network none -u 0 -w /opt/coco_ws "$IMAGE" \
      rosdep check --from-paths src --ignore-src --rosdistro jazzy -t "$t" \
      > "$d/rosdep_$t.txt" 2>&1
    grep 'All system dependencies have been satisfied' "$d/rosdep_$t.txt" >/dev/null || rc=1
  done
  "${D[@]}" run --rm --network none "$IMAGE" python3 /opt/coco/pip/check_pip_lock.py \
    /opt/coco/pip/pip-constraints.txt > "$d/pip_lock.txt" 2>&1 || rc=1
  for f in dpkg pip; do
    "${D[@]}" run --rm --network none --entrypoint cat "$IMAGE" "/opt/coco/manifest/$f.txt" > "$d/$f.txt"
  done
  [ "$rc" = 0 ] || capture_failure dependencies "$rc" "rosdep check / pip lock"
  return "$rc"
}

st_tests() {
  local rc=0 r d
  for r in $(seq 1 "$TEST_REPS"); do
    d="$OUT/tests/rep$r"; mkdir -p "$d"; chmod 0777 "$d"
    uptime > "$d/load_before.txt"
    "${D[@]}" run --rm --network none "${HARDEN[@]}" -v "$d:/out" -e COCO_TEST_OUT=/out/junit \
      "$IMAGE" test --jobs "$JOBS" > "$d/totals.json" 2> "$d/stderr.log"
    local trc=$?
    uptime > "$d/load_after.txt"
    log "   rep $r: $(cat "$d/totals.json") rc=$trc"
    if [ "$trc" != 0 ]; then
      rc=1; capture_failure "tests-rep$r" "$trc" "$IMAGE test --jobs $JOBS"
      cp -r "$d/junit" "$OUT/failures/tests-rep$r/" 2>/dev/null
    fi
  done
  return "$rc"
}

st_platform() {
  local rc=0 r
  for r in $(seq 1 "$SIM_REPS"); do
    "$HERE/platform_smoke.sh" --image "$IMAGE" --out "$OUT/platform/rep$r" \
      --timeout "$HEALTH_TIMEOUT" --project "cocoval$r" "${SMOKE_FLAGS[@]}" || rc=1
    [ "$rc" = 0 ] || cp -r "$OUT/platform/rep$r" "$OUT/failures/platform-rep$r" 2>/dev/null
  done
  return "$rc"
}

run_stage preflight st_preflight
run_stage identity st_identity
run_stage environment st_environment
run_stage dependencies st_dependencies
run_stage tests st_tests
[ "$SIM" = 1 ] && run_stage platform st_platform

python3 - "$OUT" "$IMAGE" "$STAMP" <<'EOF'
import glob, json, os, sys
out, image, stamp = sys.argv[1:4]
stages = [json.loads(l) for l in open(f'{out}/stages.jsonl')]
tests = []
for f in sorted(glob.glob(f'{out}/tests/rep*/junit/summary.json')):
    s = json.load(open(f))
    tests.append({'rep': f.split('/')[-3], 'totals': s['totals'],
                  'wall_seconds': s['wall_seconds'],
                  'packages': {p['package']: {k: p[k] for k in
                               ('passed', 'failed', 'errors', 'skipped', 'seconds', 'rc')}
                               for p in s['packages']}})
info = {}
try:
    info = json.load(open(f'{out}/identity/info.json'))
except Exception:
    pass
summary = {'image': image, 'started_utc': stamp, 'build_info': info,
           'stages': stages, 'tests': tests,
           'result': 'PASS' if all(s['rc'] == 0 for s in stages) else 'FAIL'}
json.dump(summary, open(f'{out}/validation.json', 'w'), indent=1)
print(json.dumps({'result': summary['result'],
                  'stages': {s['stage']: s['rc'] for s in stages}}))
sys.exit(0 if summary['result'] == 'PASS' else 1)
EOF
RESULT=$?
log "evidence: $OUT"
exit "$RESULT"
