#!/usr/bin/env bash
WT="/home/gautham/ros2_ws(personal)/src/coco-robot-ros2/.claude/worktrees/c2nav0-diagnosis"
R=/home/gautham/coco_nav_runs
S="$WT/docs/data/c2nav42_residual.py"
D="$WT/docs/data"
A3="$R/baseline/baseline_r01 $R/baseline/baseline_r02 $R/baseline/baseline_r03"
A4="$R/baseline/baseline_r04"
BP="$R/baseline_topology_b/baseline_topology_b_r01 $R/baseline_topology_b/baseline_topology_b_r02 $R/baseline_topology_b/baseline_topology_b_r04"
BF="$R/baseline_topology_b/baseline_topology_b_r05 $R/baseline_topology_b/baseline_topology_b_r06 $R/baseline_topology_b/baseline_topology_b_r07"
B8="$R/baseline_topology_b/baseline_topology_b_r08"
{
  echo '{'
  echo '"A_c2nav39_r01_r03":'; python3 -P "$S" trace $A3; echo ','
  echo '"A_c2nav42_r04":'; python3 -P "$S" trace $A4; echo ','
  echo '"B_prefix_c2nav41_r01_r02_r04":'; python3 -P "$S" trace $BP; echo ','
  echo '"B_fixed_r05_r07":'; python3 -P "$S" trace $BF; echo ','
  echo '"B_fixed_instrumented_r08":'; python3 -P "$S" trace $B8
  echo '}'
} > "$D/c2nav42_residual_trace.json"
{
  echo '{'
  echo '"A_c2nav39_r01_r03":'; python3 -P "$S" stall $A3; echo ','
  echo '"B_prefix_c2nav41_r01_r02_r04":'; python3 -P "$S" stall $BP; echo ','
  echo '"B_fixed_r05_r07":'; python3 -P "$S" stall $BF
  echo '}'
} > "$D/c2nav42_residual_stall.json"
{
  echo '{'
  echo '"tour_r08_recorder":'; python3 -P "$S" messages "$R/c2nav42/tour_rec_r01"; echo ','
  echo '"controlled_stop_fixed":'; python3 -P "$S" messages "$R/c2nav42/live_b_r01/stop"; echo ','
  echo '"controlled_spin_fixed":'; python3 -P "$S" messages "$R/c2nav42/live_b_r01/spin"
  echo '}'
} > "$D/c2nav42_residual_messages.json"
for f in trace stall messages; do
  python3 -c "import json,sys; json.load(open(sys.argv[1])); print('valid', sys.argv[1])" "$D/c2nav42_residual_$f.json"
done
python3 -c "
import json
t=json.load(open('$D/c2nav42_residual_trace.json'))
for k,v in t.items(): print(k, v['total'])
print(json.load(open('$D/c2nav42_residual_stall.json')))
print(json.load(open('$D/c2nav42_residual_messages.json')))
"
