#!/usr/bin/env bash
B=/home/gautham/.claude/jobs/d213ad33/tmp/isaac_probe
PY=${1:-/home/gautham/isaac-sim/venv/bin/python}
TAG=${2:-v45}
for r in 640x480 1280x720; do
  echo "=== $r"
  bash $B/run_probe.sh ${TAG}_res_$r "$PY" $B/render_probe.py 400 OMNI_KIT_ACCEPT_EULA=YES \
    PROBE_RES=$r PROBE_FRAMES=100 PROBE_WARMUP_S=240 ${EXTRA_HOME:+HOME=$EXTRA_HOME} 2>&1 | grep -aE 'exit=|vram|Maximum|FIRST|NO RGB|RESULT|LLVM|Fatal'
done
