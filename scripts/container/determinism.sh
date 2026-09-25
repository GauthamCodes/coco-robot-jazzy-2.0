#!/usr/bin/env bash
# determinism.sh --image TAG --ref REF --out DIR — is a --no-cache rebuild
# of the same commit the same image?
#
# Builds REF clean-room with --no-cache (the base image is not re-pulled:
# its digest is pinned and it is already local), then compares the two
# images layer by layer (diff_id). For every layer that differs it asks the
# next question: do the FILE CONTENTS differ, or only the metadata (mtimes,
# which every RUN stamps with the wall clock)? It hashes file contents,
# ignoring timestamps, under the trees each COCO layer writes, in both
# images. Writes DIR/determinism.json and removes the rebuilt image.
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMAGE=""; REF=HEAD; OUT=""
while [ $# -gt 0 ]; do
  case "$1" in
    --image) IMAGE=$2; shift ;;
    --ref) REF=$2; shift ;;
    --out) OUT=$2; shift ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
  shift
done
[ -n "$IMAGE" ] && [ -n "$OUT" ] || { echo "--image and --out are required" >&2; exit 2; }
read -r -a D <<< "${DOCKER:-docker}"
mkdir -p "$OUT"
NEW="${IMAGE}-nocache"
"$HERE/build.sh" --ref "$REF" --tag "$NEW" --no-cache --out "$OUT/build" || exit 1
for t in "$IMAGE" "$NEW"; do
  "${D[@]}" image inspect -f '{{range .RootFS.Layers}}{{println .}}{{end}}' "$t" \
    | sed '/^$/d' > "$OUT/layers_$(echo "$t" | tr '/:' '__').txt"
done
"${D[@]}" history --no-trunc --format '{{.Size}}|{{.CreatedBy}}' "$IMAGE" | tac \
  | awk -F'|' '$1!="0B"' > "$OUT/history.txt"
# Content hashes (path + sha256 of bytes; no mtimes) of what COCO's layers write.
ROOTS="/usr/local/lib/python3.12/dist-packages /opt/coco_ws /opt/coco /opt/ros/jazzy/share/nav2_bringup /var/lib/dpkg/status /etc/passwd"
for t in "$IMAGE" "$NEW"; do
  "${D[@]}" run --rm --network none -u 0 --entrypoint bash "$t" -c \
    "find $ROOTS -type f -print0 2>/dev/null | sort -z | xargs -0 sha256sum" \
    > "$OUT/content_$(echo "$t" | tr '/:' '__').txt" 2>/dev/null
done
python3 - "$OUT" "$IMAGE" "$NEW" <<'EOF'
import json, sys
out, a, b = sys.argv[1:4]
fn = lambda t: t.replace('/', '_').replace(':', '_')
la = open(f'{out}/layers_{fn(a)}.txt').read().split()
lb = open(f'{out}/layers_{fn(b)}.txt').read().split()
hist = [l.rstrip('\n').split('|', 1) for l in open(f'{out}/history.txt')]
layers = []
for i, (x, y) in enumerate(zip(la, lb)):
    size, cmd = hist[i] if i < len(hist) else ('?', '?')
    layers.append({'index': i + 1, 'same': x == y, 'size': size,
                   'instruction': cmd[:100], 'diff_id_a': x[:19], 'diff_id_b': y[:19]})
def content(t):
    d = {}
    for line in open(f'{out}/content_{fn(t)}.txt'):
        h, _, p = line.rstrip('\n').partition('  ')
        d[p] = h
    return d
ca, cb = content(a), content(b)
changed = sorted(p for p in ca.keys() & cb.keys() if ca[p] != cb[p])
res = {'image_cached': a, 'image_nocache': b,
       'layers_total': len(la), 'layers_identical': sum(l['same'] for l in layers),
       'layers': layers,
       'content_files_compared': len(ca.keys() & cb.keys()),
       'content_only_in_cached': len(ca.keys() - cb.keys()),
       'content_only_in_nocache': len(cb.keys() - ca.keys()),
       'content_files_differing': len(changed),
       'content_differing_examples': changed[:40]}
json.dump(res, open(f'{out}/determinism.json', 'w'), indent=1)
print(json.dumps({k: v for k, v in res.items() if k not in ('layers', 'content_differing_examples')}))
EOF
"${D[@]}" rmi "$NEW" >/dev/null
