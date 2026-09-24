#!/usr/bin/env bash
# usage: ros_run.sh <label> <isaac_python> <bridge_lib_dir> <stage> <seconds> [extra KEY=VAL for Isaac...]
LABEL=$1; PY=$2; BRLIB=$3; STAGE=$4; SECS=$5; shift 5
BASE=/home/gautham/.claude/jobs/d213ad33/tmp/isaac_probe
OUT=$BASE/runs/$LABEL
DOM=77
IRMW=${ISAAC_RMW:-rmw_fastrtps_cpp}
JRMW=${JAZZY_RMW:-rmw_fastrtps_cpp}
bash $BASE/run_probe.sh "$LABEL" "$PY" $BASE/ros_probe.py $((SECS + 240)) \
  OMNI_KIT_ACCEPT_EULA=YES ROS_DISTRO=humble RMW_IMPLEMENTATION=$IRMW \
  ROS_DOMAIN_ID=$DOM LD_LIBRARY_PATH="$BRLIB" PROBE_STAGE=$STAGE PROBE_SECONDS=$SECS "$@" \
  > $BASE/runs/$LABEL.isaac.out 2>&1 &
ISAAC=$!
for i in $(seq 1 240); do [ -f "$OUT/ready.marker" ] && break; sleep 1; done
if [ ! -f "$OUT/ready.marker" ]; then echo "Isaac never became ready"; wait $ISAAC; cat $BASE/runs/$LABEL.isaac.out; exit 2; fi
echo "Isaac ready after ${i}s"
if [ -n "$NO_JAZZY" ]; then wait $ISAAC; cat $BASE/runs/$LABEL.isaac.out; exit 0; fi
echo "starting Jazzy side (RMW $JRMW)"
cat > "$OUT/jazzy_env.sh" <<EOF
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=$DOM RMW_IMPLEMENTATION=$JRMW IMG_RELIABLE=$IMG_RELIABLE
env | sort > $OUT/jazzy_env.txt
cd $BASE && python3 jazzy_side.py $((SECS - 15)) $OUT/jazzy_result.json
EOF
env -i HOME=/home/gautham PATH=/usr/bin:/bin bash --noprofile --norc "$OUT/jazzy_env.sh" > "$OUT/jazzy.out" 2>&1
echo "jazzy exit=$?"
wait $ISAAC
cat $BASE/runs/$LABEL.isaac.out
echo "===== JAZZY"
python3 -c "
import json;r=json.load(open('$OUT/jazzy_result.json'))
for k,v in r['topics'].items(): print('%-6s n=%-5d first=%s last=%s'%(k,v['n'],v['t_first'],v['t_last']))
print('twist_published',r['twist_published'])
for k,v in r['samples'].items(): print(' ',k,v)
print('graph:',r['graph_topics'])" 2>&1 || tail -30 "$OUT/jazzy.out"
grep -E '^(ROS_DISTRO|RMW_IMPLEMENTATION|ROS_DOMAIN_ID|AMENT_PREFIX_PATH|LD_LIBRARY_PATH|PYTHONPATH)=' "$OUT/jazzy_env.txt" | cut -c1-200
