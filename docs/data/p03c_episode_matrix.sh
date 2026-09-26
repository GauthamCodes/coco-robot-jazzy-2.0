#!/usr/bin/env bash
# P0.3 stage C: the first episode matrix. One fresh simulator per run.
#
#   COCO_WS=$HOME/coco_p03c_ws bash docs/data/p03c_episode_matrix.sh ROOT
#
# FIXED   seed 0, all four colours -- the default command lines.
# COLOUR  seeds 1, 2, 4.   POSITION seeds 1, 2, 4.
#
# The seeds and the requested colours were fixed by a rule stated BEFORE any
# of these ran, not picked from results:
#   - the first three seeds >= 1 whose colour permutation is distinct
#     (seeds 1 and 3 draw the same permutation, so 3 is skipped);
#   - for each, request the colour the permutation moved FARTHEST from its
#     frozen lane (ties to TARGET_COLOURS order), because a colour left in
#     its own lane is one the pre-episode mission would also have fetched,
#     and the run would not test the region mapping at all.
# That gives yellow (seed 1), red (seed 2), green (seed 4) at both levels:
# the same assignment, with and without the positions jitter.
#
# Modes are INTERLEAVED so machine drift over a multi-hour sweep is spread
# across them instead of being confounded with one (C2-NAV.46's reason for
# interleaving colours). A run that already has a result is skipped.
set +u
WT="${COCO_WT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
ROOT="${1:-$HOME/coco_nav_runs/p03c_matrix}"
mkdir -p "$ROOT"
RUNS=(
    "fixed_red fixed 0 red"
    "colours_s1 colours 1 yellow"
    "positions_s1 positions 1 yellow"
    "fixed_green fixed 0 green"
    "colours_s2 colours 2 red"
    "positions_s2 positions 2 red"
    "fixed_blue fixed 0 blue"
    "colours_s4 colours 4 green"
    "positions_s4 positions 4 green"
    "fixed_yellow fixed 0 yellow"
)
for spec in "${RUNS[@]}"; do
    read -r name level seed colour <<< "$spec"
    RUN="$ROOT/$name"
    if [ -f "$RUN/result.json" ]; then
        echo "=== $name already has a result, skipping"
        continue
    fi
    echo "=== p03c $name ($level seed=$seed colour=$colour) starting $(date -u +%H:%M:%SZ)"
    bash "$WT/docs/data/p03c_episode_run.sh" "$RUN" "$level" "$seed" "$colour"
    echo "=== $name runner exit $? at $(date -u +%H:%M:%SZ)"
    cat "$RUN/final_state.txt" 2>/dev/null
    sleep 20
done
echo "=== p03c matrix done $(date -u +%H:%M:%SZ)"
