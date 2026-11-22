#!/usr/bin/env bash
# The gazebo_models unit tests, from inside the package, clean graph.
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/env.sh"
cd "$HERE/../gazebo_models" || exit 1
python3 -m pytest test -q 2>&1 | tail -15
