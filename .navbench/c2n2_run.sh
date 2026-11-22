#!/usr/bin/env bash
# C2-NAV.2: three enclosure_entry repeats, topology A, 75 s timeout.
#
# Invoked as python3 on the WORKTREE script rather than
# `ros2 run gazebo_models nav_bench.py`, because nav_bench.py was added by
# the C2-NAV.0 commit and install/gazebo_models symlinks to the trunk,
# which is at main and predates it -- `ros2 run` finds no such executable.
# This is the identical file and the identical invocation that produced the
# C2-NAV.0 baseline and the C2-NAV.1 experiment (.navbench/run.sh), so the
# three runs are tool-for-tool comparable.
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/env.sh"
python3 "$HERE/../gazebo_models/scripts/nav_bench.py" \
    --tag navA_baseobs --repeats 3 --timeout 75 \
    --only enclosure_entry --out "$HERE/results"
