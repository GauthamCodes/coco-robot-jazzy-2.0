#!/usr/bin/env bash
# usage: run_probe.sh <label> <python> <script> <timeout_s> [KEY=VAL ...]
# Clean env (env -i); only HOME, PATH and the named KEY=VALs reach Isaac.
LABEL=$1; PY=$2; SCRIPT=$3; TMO=$4; shift 4
BASE=/home/gautham/.claude/jobs/d213ad33/tmp/isaac_probe
OUT=$BASE/runs/$LABEL
rm -rf "$OUT"; mkdir -p "$OUT"
cd "$OUT" || exit 9
ENVV=(HOME=/home/gautham PATH=/usr/bin:/bin PROBE_LOG=$OUT "$@")
env -i "${ENVV[@]}" bash -c 'env | sort' > env.txt
{ echo "--- filtered"; for k in ROS_DISTRO AMENT_PREFIX_PATH COLCON_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH RMW_IMPLEMENTATION ROS_DOMAIN_ID VK_ICD_FILENAMES; do
    echo "$k=$(grep "^$k=" env.txt | cut -d= -f2-)"; done; } >> env.txt
nvidia-smi --query-gpu=timestamp,memory.used,utilization.gpu --format=csv,noheader -l 1 > vram.csv 2>&1 &
SMI=$!
free -m | awk '/Mem/{print "ram_used_before_MiB="$3" avail="$7}' > ram.txt
T=$(date +%s.%N)
env -i "${ENVV[@]}" timeout -s KILL "$TMO" /usr/bin/time -v "$PY" -u "$SCRIPT" > log.txt 2>&1
RC=$?
echo "exit=$RC wall=$(echo "$(date +%s.%N) - $T" | bc)" | tee rc.txt
kill $SMI 2>/dev/null
echo "vram_peak_MiB=$(cut -d, -f2 vram.csv | grep -oE '[0-9]+' | sort -n | tail -1) vram_idle_MiB=$(head -1 vram.csv | cut -d, -f2)" | tee -a rc.txt
grep -aE 'Maximum resident|Elapsed \(wall|Exit status' log.txt | tee -a rc.txt
grep -aE 'PROBE|LLVM|out of memory|Segmentation|Aborted|Traceback|Error' log.txt | grep -av '\[Warning\]' | head -60
