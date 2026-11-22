#!/usr/bin/env bash
# C2-NAV.25 live matrix: the SAME driver as C2-NAV.21/.23, with the three
# specs of this experiment baked in so nothing about the route, the
# timeouts or the through-poses can drift between the arms.
#
#   topology A  c2n25_slow_r1
#   topology A  c2n25_slow_r2
#   topology B  c2n25_bslow_r1
#
# One Gazebo at a time, a fresh simulator per run, ros_clean between.
# The ONLY behavioural difference from the frozen baseline arm is the
# parameter file: c2nav25_slow_params.yaml, which is
# c2nav11_ntp_params.yaml with PolygonSlow.slowdown_ratio 0.3 -> 1.0 and
# nothing else (asserted leaf-by-leaf by c2nav25_slow.py paramdiff).
#
# Naming: no string here contains "nav2_", which ros_clean.sh's 'nav[2]_'
# pattern would match.
HERE="$(cd "$(dirname "$0")" && pwd)"
WT="$(cd "$HERE/.." && pwd)"
PARAMS="$WT/docs/data/c2nav25_slow_params.yaml"

exec bash "$HERE/c2n21_matrix.sh" \
    "c2n25_slow_r1:${PARAMS}:A" \
    "c2n25_slow_r2:${PARAMS}:A" \
    "c2n25_bslow_r1:${PARAMS}:B"
