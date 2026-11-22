#!/usr/bin/env bash
# C2-NAV.8: the affected package's tests, on a clean ROS graph.
#
# cwd MUST be the package directory (CLAUDE.md rule 8.1: from the repo
# root the coco_rl/ directory shadows the installed module), and
# gazebo_models needs --ignore=test_integration or pytest dies importing
# test_sim_bringup.launch.py during COLLECTION and silently reports 0
# tests rather than failing loudly.
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/env.sh"
WT="$(cd "$HERE/.." && pwd)"
cd "$WT/gazebo_models" || exit 1
echo "cwd: $(pwd)"
python3 -m pytest -q --ignore=test_integration 2>&1 | tail -8
