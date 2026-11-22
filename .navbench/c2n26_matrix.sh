#!/usr/bin/env bash
# C2-NAV.26 live matrix: SIX fresh tours of the C2-NAV.25 candidate and
# NOTHING ELSE CHANGED. Same driver (c2n21_matrix.sh), same runners, same
# route, same timeouts, same through-poses, same parameter file
# (c2nav25_slow_params.yaml, sha256 4c15893e...). The only difference
# from C2-NAV.25 is the number of independent trials.
#
#   topology A  c2n26_slow_r1   topology B  c2n26_bslow_r1
#   topology A  c2n26_slow_r2   topology B  c2n26_bslow_r2
#   topology A  c2n26_slow_r3   topology B  c2n26_bslow_r3
#
# INTERLEAVED A/B/A/B/A/B rather than AAABBB, for the reason
# c2n21_matrix.sh's own header gives: a block ordering hands one arm the
# first half-hour of machine state and the other the second, and here the
# two arms are the two topologies. Topology is the thing this session is
# asked to compare, so it is the thing that must not be confounded with
# time.
#
# One Gazebo at a time, a fresh simulator per run, ros_clean between --
# all inherited from the driver, none of it re-implemented here.
#
# Naming: no string in this file contains the pattern ros_clean.sh
# sweeps for ('nav[2]_'), which would match any command line carrying it.
HERE="$(cd "$(dirname "$0")" && pwd)"
WT="$(cd "$HERE/.." && pwd)"
PARAMS="$WT/docs/data/c2nav25_slow_params.yaml"

exec bash "$HERE/c2n21_matrix.sh" \
    "c2n26_slow_r1:${PARAMS}:A" \
    "c2n26_bslow_r1:${PARAMS}:B" \
    "c2n26_slow_r2:${PARAMS}:A" \
    "c2n26_bslow_r2:${PARAMS}:B" \
    "c2n26_slow_r3:${PARAMS}:A" \
    "c2n26_bslow_r3:${PARAMS}:B"
