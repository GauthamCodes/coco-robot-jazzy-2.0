#!/usr/bin/env bash
# gazebo_models is the only package C2-NAV.3 touches (ros_clean.sh), so it
# is the package whose tests must be green. CLAUDE.md rule 8: cwd is the
# PACKAGE directory, --ignore=test_integration or pytest dies during
# collection and silently reports 0 tests, and the ROS graph must be
# clean. Baseline for this package is 41.
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/env.sh"
cd "$HERE/../gazebo_models" || exit 1
echo "cwd: $(pwd)"
python3 -m pytest -q --ignore=test_integration 2>&1 | tail -20
