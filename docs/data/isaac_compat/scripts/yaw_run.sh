#!/usr/bin/env bash
cd /home/gautham/.claude/jobs/d213ad33/tmp/isaac_probe || exit 9
PY=${1:-/home/gautham/isaac-sim/venv/bin/python}
for m in ground groundice air; do
  env -i HOME=/home/gautham PATH=/usr/bin:/bin OMNI_KIT_ACCEPT_EULA=YES timeout 200 "$PY" -u yaw_probe.py $m > runs/yaw_$m.log 2>&1
  grep -a '^MODE' runs/yaw_$m.log
done
