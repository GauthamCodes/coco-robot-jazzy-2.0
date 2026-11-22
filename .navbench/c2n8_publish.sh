#!/usr/bin/env bash
# C2-NAV.8: copy the run artifacts out of the uncommitted scratch
# directory into docs/data, so every number in the record is
# reproducible from committed files. Same arrangement as C2-NAV.6 and
# C2-NAV.7 (c2nav7_cand_r*_stop.{csv,json}).
HERE="$(cd "$(dirname "$0")" && pwd)"
WT="$(cd "$HERE/.." && pwd)"
SRC="$HERE/results"
DST="$WT/docs/data"

for r in r1 r2 r3; do
    cp -f "$SRC/c2n8_tour_${r}_stop.csv"  "$DST/c2nav8_tour_${r}_stop.csv"
    cp -f "$SRC/c2n8_tour_${r}_stop.json" "$DST/c2nav8_tour_${r}_stop.json"
done
# The live parameter read-back. All three tours produced one; r1's is
# copied and the other two are DIFFED against it below, so "no parameter
# moved between tours" is a diff rather than a claim.
cp -f "$HERE/logs/c2n6_params_c2n8_tour_r1.txt" "$DST/c2nav8_params_live.txt"

echo "=== live read-back identical across all three tours? ==="
for r in r2 r3; do
    if diff -q "$HERE/logs/c2n6_params_c2n8_tour_r1.txt" \
               "$HERE/logs/c2n6_params_c2n8_tour_${r}.txt" >/dev/null; then
        echo "  r1 vs $r: IDENTICAL"
    else
        echo "  r1 vs $r: DIFFERS --"
        diff "$HERE/logs/c2n6_params_c2n8_tour_r1.txt" \
             "$HERE/logs/c2n6_params_c2n8_tour_${r}.txt"
    fi
done
echo "=== and identical to C2-NAV.7's committed read-back? ==="
if diff -q "$DST/c2nav7_params_live_cand.txt" "$DST/c2nav8_params_live.txt" \
        >/dev/null; then
    echo "  C2-NAV.7 vs C2-NAV.8: IDENTICAL -- no navigation parameter moved"
else
    diff "$DST/c2nav7_params_live_cand.txt" "$DST/c2nav8_params_live.txt"
fi
echo "=== published ==="
ls -la "$DST"/c2nav8_*
