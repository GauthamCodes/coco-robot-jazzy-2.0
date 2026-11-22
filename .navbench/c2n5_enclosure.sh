#!/usr/bin/env bash
# C2-NAV.5 stage 1: fresh enclosure_entry replication, N=5 per condition.
#
# BASELINE  = C2-NAV.0 exactly, cost_scaling_factor 5.0
#             docs/data/c2nav3_baseline_params.yaml  sha256 dbcee9ca...
# CANDIDATE = that file plus ONE line, cost_scaling_factor 65.0
#             docs/data/c2nav4_csf65_params.yaml     sha256 3d9623d6...
#
# INTERLEAVED, not blocked: base r1, csf65 r1, base r2, csf65 r2, ...
# Fifty minutes of runs on a machine that also renders is long enough for
# thermal or memory drift to matter, and a blocked design would charge all
# of it to whichever condition ran second.
#
# 150 s per leg, both conditions. That is C2-NAV.4's equal-budget stage,
# chosen because a TIMEOUT at 75 s is a statement about the controller AND
# the clock: the baseline's own 150 s run drives 5.228 m rather than
# 2.8 m, so the shorter budget would flatter the candidate.
HERE="$(cd "$(dirname "$0")" && pwd)"
D="$(cd "$HERE/../docs/data" && pwd)"
mkdir -p "$HERE/logs" "$HERE/results"
N="${1:-5}"

BASE="$D/c2nav3_baseline_params.yaml"
CAND="$D/c2nav4_csf65_params.yaml"

for i in $(seq 1 "$N"); do
    for cond in base csf65; do
        [ "$cond" = base ] && P="$BASE" || P="$CAND"
        TAG="c2n5_enc_${cond}_r${i}"
        echo "######## $TAG  $(date -u +%H:%M:%S) ########"
        bash "$HERE/c2n5_run.sh" "$P" "$TAG" enclosure_entry 150 \
            > "$HERE/logs/$TAG.out" 2>&1
        echo "   exit $?  -> logs/$TAG.out"
        grep -E '^\[nav_bench\]|cost_scaling|bench exit' \
            "$HERE/logs/$TAG.out" | tail -6
    done
done
echo "######## ENCLOSURE STAGE DONE $(date -u +%H:%M:%S) ########"
