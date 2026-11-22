#!/usr/bin/env bash
# run.sh <tag> <repeats> [extra args...]
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/env.sh"
TAG="${1:-smoke}"; shift
REP="${1:-1}"; shift
python3 "$HERE/../gazebo_models/scripts/nav_bench.py" \
    --tag "$TAG" --repeats "$REP" --out "$HERE/results" "$@"
