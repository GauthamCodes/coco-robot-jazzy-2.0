#!/usr/bin/env bash
# Emit each nav_bench leg result once, exit when the JSON is written.
OUT="$1"
prev="/tmp/.nb_prev_$$"
: > "$prev"
while true; do
    grep -aE 'SUCCEEDED|ABORTED|CANCELED|TIMEOUT|no_result|Traceback|rror|wrote ' "$OUT" 2>/dev/null > "$prev.new"
    comm -13 "$prev" "$prev.new" 2>/dev/null
    mv "$prev.new" "$prev"
    if grep -qa 'nav_bench] wrote' "$OUT" 2>/dev/null; then break; fi
    sleep 20
done
rm -f "$prev" "$prev.new"
