#!/usr/bin/env bash
# C2-NAV.26 wrapper: source the environment from a FILE (never a -c string)
# and run an analysis module from the docs/data directory it lives in, so
# sys.path[0] is a directory we control (CLAUDE.md's shared-scratch trap).
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/env.sh"
WT="$(cd "$HERE/.." && pwd)"
cd "$WT/docs/data" || exit 1
exec python3 "$@"
