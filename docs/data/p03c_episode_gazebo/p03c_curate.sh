#!/usr/bin/env bash
# Copy the small, reviewable files of each p03c run into this directory.
#
#   bash docs/data/p03c_episode_gazebo/p03c_curate.sh ~/coco_nav_runs/p03c_matrix
#
# Kept: the manifest, what gz spawned and read back, what the robot side was
# told, the final state, the EpisodeResult, the runner's checks, the command-
# path summary. Left in ROOT (too large to commit): sim.log, mission.log,
# cmdpath/trace.csv, hrec.csv, state_stream.txt.
set -o pipefail
ROOT="${1:?usage: p03c_curate.sh ROOT}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
for run in "$ROOT"/*/; do
    name="$(basename "$run")"
    dest="$HERE/matrix/$name"
    mkdir -p "$dest"
    for f in manifest.json spawned_manifest.json mission_inputs.json \
             readback_spawn.json readback_end.json final_gz.json \
             region_params.txt final_state.txt result.json meta.txt \
             start.txt lifecycle.txt topology_live.txt depth_off.txt; do
        [ -f "$run/$f" ] && cp "$run/$f" "$dest/"
    done
    [ -f "$run/cmdpath/summary.json" ] && cp "$run/cmdpath/summary.json" "$dest/cmdpath_summary.json"
    grep -E 'p03c_run: (PASS|FAIL|wall|torn|mission terminal)|NOT reached|INCOMPLETE' \
        "$run/runner.log" > "$dest/runner_checks.txt" 2>/dev/null
    grep -E 'mission_executive up|cross-track datum|\[approach_server\]: (arrived|approach finished)|\[episode\]' \
        "$run/mission.log" "$run/sim.log" 2>/dev/null \
        | sed "s#$run##" > "$dest/key_log_lines.txt"
done
