#!/usr/bin/env bash
# C2-NAV.26: block until the six-tour matrix finishes, then exit. One
# shot, hard-capped, no recurrence -- this is a wait, not a monitor: it
# starts nothing, changes nothing, and cannot outlive the cap.
HERE="$(cd "$(dirname "$0")" && pwd)"
CAP=${1:-5400}
end=$(( $(date +%s) + CAP ))
while [ "$(date +%s)" -lt "$end" ]; do
    if grep -q 'matrix complete' "$HERE/logs/c2n26_matrix.out" 2>/dev/null; then
        echo "MATRIX COMPLETE"
        exit 0
    fi
    if ! pgrep -f 'c2n2._matri[x].sh' >/dev/null 2>&1; then
        echo "MATRIX DRIVER GONE (no 'matrix complete' line)"
        exit 3
    fi
    sleep 20
done
echo "CAP ${CAP}s REACHED"
exit 4
