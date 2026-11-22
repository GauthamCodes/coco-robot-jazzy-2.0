#!/usr/bin/env bash
# C2-NAV.8: do the tables read from the COMMITTED artifact match the
# tables read from the scratch directory, byte for byte?
#
# C2-NAV.5 and C2-NAV.6 both did this with `diff`, and for the same
# reason: `collect` folds an uncommitted scratch directory into one
# committed JSON, and if the two sources disagree the record cannot be
# reproduced from the repository alone.
#
# The scratch run must be forced to read the scratch CSVs and the
# committed run the committed ones; that is what the directory argument
# does. Run from a directory this experiment owns.
HERE="$(cd "$(dirname "$0")" && pwd)"
WT="$(cd "$HERE/.." && pwd)"
R="$WT/docs/data/c2nav8_report.py"
T=$(mktemp -d)
bad=0
for mode in legs entry exit stop clear compare; do
    python3 "$R" "$mode" "$HERE/results"                > "$T/scratch_$mode.txt" 2>&1
    python3 "$R" "$mode" "$WT/docs/data/c2nav8_bench.json" > "$T/committed_$mode.txt" 2>&1
    # The first line is the source path and is expected to differ.
    if diff <(tail -n +2 "$T/scratch_$mode.txt") \
            <(tail -n +2 "$T/committed_$mode.txt") >/dev/null; then
        echo "ok    $mode: committed artifact reproduces the scratch tables"
    else
        echo "FAIL  $mode:"
        diff <(tail -n +2 "$T/scratch_$mode.txt") \
             <(tail -n +2 "$T/committed_$mode.txt") | head -20
        bad=1
    fi
done
rm -rf "$T"
exit $bad
