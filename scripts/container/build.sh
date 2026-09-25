#!/usr/bin/env bash
# build.sh — build a COCO image whose content is a function of a commit.
#
#   scripts/container/build.sh                       # HEAD -> coco-platform:<sha12>
#   scripts/container/build.sh --ref p03-episode-spec
#   scripts/container/build.sh --ref main --infra-ref HEAD --tag coco-platform:main
#   scripts/container/build.sh --worktree            # the working tree, dirty or not
#
# Default (clean room): the build context is `git archive REF`, so nothing
# the working tree happens to hold -- a host build/ tree, __pycache__, an
# untracked experiment, another worktree -- can reach the image, and the
# image records exactly which commit it is.
#
# --infra-ref REF2 takes the container files (Dockerfile, .dockerignore,
# docker/, scripts/ci/, scripts/container/) from REF2 and everything else
# from REF. That is how an older commit that predates this Dockerfile is
# built with it; the image then records BOTH shas.
#
# Build metadata (wall clock, duration, docker version, per-step timings)
# is written OUTSIDE the image, to --out (default
# $COCO_EVIDENCE_DIR/builds/<tag-safe>), as build-meta.json + build.log.
# Wall-clock time is never baked into the image: the commit time is passed
# as SOURCE_DATE_EPOCH, which BuildKit uses for the image's created stamp.
#
# DOCKER overrides the client (e.g. DOCKER="sg docker -c docker"; a session
# that joined the docker group after login needs that).
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
REF=HEAD; INFRA=""; TAG=""; OUT=""; MODE=git; EXTRA=()
while [ $# -gt 0 ]; do
  case "$1" in
    --ref) REF=$2; shift ;;
    --infra-ref) INFRA=$2; shift ;;
    --tag) TAG=$2; shift ;;
    --out) OUT=$2; shift ;;
    --worktree) MODE=worktree ;;
    --no-cache|--pull) EXTRA+=("$1") ;;
    --build-arg) EXTRA+=("--build-arg" "$2"); shift ;;
    -h|--help) sed -n '2,30p' "$0"; exit 0 ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
  shift
done

git_() { git -C "$REPO" "$@"; }
SHA=$(git_ rev-parse --verify "$REF^{commit}")
EPOCH=$(git_ log -1 --format=%ct "$SHA")
# A branch name when REF is one (or is HEAD on one); otherwise the nearest
# local branch name-rev can describe the commit by ("main~2"), else "unknown".
if git_ show-ref --verify --quiet "refs/heads/$REF"; then BRANCH=$REF
elif [ "$REF" = HEAD ]; then BRANCH=$(git_ rev-parse --abbrev-ref HEAD)
else BRANCH=$(git_ name-rev --name-only --refs='refs/heads/*' "$SHA" 2>/dev/null || true)
fi
case "$BRANCH" in ''|undefined) BRANCH=unknown ;; esac
INFRA_SHA=""
[ -n "$INFRA" ] && INFRA_SHA=$(git_ rev-parse --verify "$INFRA^{commit}")
TAG=${TAG:-coco-platform:${SHA:0:12}}
SAFE=$(echo "$TAG" | tr '/:' '__')
OUT=${OUT:-${COCO_EVIDENCE_DIR:-$HOME/coco_container_evidence}/builds/$SAFE}
mkdir -p "$OUT"
read -r -a DOCKER_CMD <<< "${DOCKER:-docker}"

if [ "$MODE" = worktree ]; then
  CTX="$REPO"
  DIRTY=$([ -n "$(git_ status --porcelain)" ] && echo true || echo false)
  [ "$DIRTY" = true ] && git_ status --porcelain > "$OUT/dirty_paths.txt"
else
  CTX=$(mktemp -d "${TMPDIR:-/tmp}/coco-ctx.XXXXXX")
  trap 'rm -rf "$CTX"' EXIT
  git_ archive --format=tar "$SHA" | tar -x -C "$CTX"
  if [ -n "$INFRA_SHA" ]; then
    # Only the paths that exist at INFRA; a missing one is not an error.
    for p in Dockerfile .dockerignore docker scripts/ci scripts/container; do
      if git_ cat-file -e "$INFRA_SHA:$p" 2>/dev/null; then
        rm -rf "${CTX:?}/$p"
        git_ archive --format=tar "$INFRA_SHA" "$p" | tar -x -C "$CTX"
      fi
    done
  fi
  DIRTY=false
fi

ARGS=(--build-arg "COCO_GIT_SHA=$SHA"
      --build-arg "COCO_GIT_BRANCH=$BRANCH"
      --build-arg "COCO_GIT_DIRTY=$DIRTY"
      --build-arg "COCO_SOURCE_DATE_EPOCH=$EPOCH"
      --build-arg "SOURCE_DATE_EPOCH=$EPOCH"
      --build-arg "COCO_INFRA_SHA=${INFRA_SHA:-$SHA}")

echo "[build] $TAG <- $SHA (${BRANCH}) infra=${INFRA_SHA:-same} mode=$MODE"
START=$(date +%s.%N)
set +e
"${DOCKER_CMD[@]}" build --progress=plain -t "$TAG" "${ARGS[@]}" "${EXTRA[@]}" "$CTX" \
  > "$OUT/build.log" 2>&1
RC=$?
set -e
END=$(date +%s.%N)

IMAGE_ID=""; SIZE=""
if [ "$RC" -eq 0 ]; then
  IMAGE_ID=$("${DOCKER_CMD[@]}" image inspect -f '{{.Id}}' "$TAG")
  SIZE=$("${DOCKER_CMD[@]}" image inspect -f '{{.Size}}' "$TAG")
fi
python3 - "$OUT" <<EOF
import json, re, sys
out = sys.argv[1]
steps, names = [], {}
for line in open(f'{out}/build.log', errors='replace'):
    m = re.match(r'#(\d+) (\[.*)$', line.rstrip())
    if m and m.group(1) not in names:
        names[m.group(1)] = m.group(2)[:120]
    m = re.match(r'#(\d+) (DONE ([\d.]+)s|CACHED)', line)
    if m and m.group(1) in names:
        steps.append({'step': names[m.group(1)],
                      'seconds': float(m.group(3)) if m.group(3) else 0.0,
                      'cached': m.group(2) == 'CACHED'})
meta = {
    'kind': 'build-metadata (not part of the image)',
    'tag': '$TAG', 'rc': $RC, 'mode': '$MODE',
    'git_sha': '$SHA', 'git_branch': '$BRANCH', 'git_dirty': '$DIRTY',
    'infra_sha': '${INFRA_SHA:-$SHA}', 'source_date_epoch': $EPOCH,
    'wall_start_epoch': $START, 'wall_end_epoch': $END,
    'wall_seconds': round($END - $START, 1),
    'image_id': '$IMAGE_ID', 'image_size_bytes': '$SIZE' or None,
    'extra_args': '${EXTRA[*]:-}',
    'steps': steps,
}
json.dump(meta, open(f'{out}/build-meta.json', 'w'), indent=1)
print(json.dumps({k: meta[k] for k in ('tag', 'rc', 'wall_seconds', 'image_id')}))
EOF
exit "$RC"
