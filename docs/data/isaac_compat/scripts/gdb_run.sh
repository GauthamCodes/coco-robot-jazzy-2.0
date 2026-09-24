#!/usr/bin/env bash
# gdb_run.sh <label> <python> <bridge_lib> <stage> <seconds>
LABEL=$1; PY=$2; BRLIB=$3; STAGE=$4; SECS=$5
BASE=/home/gautham/.claude/jobs/d213ad33/tmp/isaac_probe
OUT=$BASE/runs/$LABEL; rm -rf "$OUT"; mkdir -p "$OUT"; cd "$OUT" || exit 9
cat > cmds.gdb <<'EOF'
set pagination off
set confirm off
handle SIGPIPE nostop noprint pass
handle SIG32 nostop noprint pass
handle SIG33 nostop noprint pass
handle SIG34 nostop noprint pass
handle SIGUSR1 nostop noprint pass
handle SIGUSR2 nostop noprint pass
catch throw std::bad_alloc
run
echo \n=== STOPPED: current thread bt\n
bt 40
echo \n=== info sharedlibrary (ros/dds)\n
info sharedlibrary fastrtps
info sharedlibrary rmw
kill
quit
EOF
env -i HOME=/home/gautham PATH=/usr/bin:/bin OMNI_KIT_ACCEPT_EULA=YES ROS_DISTRO=humble \
  RMW_IMPLEMENTATION=${ISAAC_RMW:-rmw_fastrtps_cpp} ROS_DOMAIN_ID=77 LD_LIBRARY_PATH="$BRLIB" \
  PROBE_LOG=$OUT PROBE_STAGE=$STAGE PROBE_SECONDS=$SECS \
  timeout -s KILL 400 gdb -batch -x cmds.gdb --args "$PY" -u $BASE/ros_probe.py > log.txt 2>&1 &
G=$!
if [ -n "$WITH_JAZZY" ]; then
  for i in $(seq 1 300); do [ -f "$OUT/ready.marker" ] && break; sleep 1; done
  echo "ready after ${i}s; jazzy (RMW ${JAZZY_RMW:-rmw_fastrtps_cpp}) starting"
  printf 'source /opt/ros/jazzy/setup.bash\nexport ROS_DOMAIN_ID=77 RMW_IMPLEMENTATION=%s\ncd %s && python3 jazzy_side.py 25 %s/jazzy_result.json\n' "${JAZZY_RMW:-rmw_fastrtps_cpp}" "$BASE" "$OUT" > jz.sh
  env -i HOME=/home/gautham PATH=/usr/bin:/bin bash --noprofile --norc jz.sh > jazzy.out 2>&1
  echo "jazzy exit=$?"
fi
wait $G
echo "exit=$?"
grep -aE 'PROBE|STOPPED|received signal|Catchpoint|LLVM' log.txt | head -20
grep -a -A40 '=== STOPPED' log.txt | grep -aE '^#' | cut -c1-220 | head -40
