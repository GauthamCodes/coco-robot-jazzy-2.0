#!/usr/bin/env bash
# refresh_pip_constraints.sh — re-resolve docker/pip-constraints.txt.
#
#   docker/refresh_pip_constraints.sh [IMAGE]      (default coco-platform:jazzy)
#
# Resolves the HARD pins (torch, stable-baselines3, gymnasium, cloudpickle,
# mujoco, tornado -- read from the current file) against the image's own apt
# Python in a throwaway root container with the pip layer removed, and
# prints the full resolved set. It does not edit anything: review the diff,
# paste it in, rebuild, and re-run scripts/container/validate.sh. Changing a
# HARD pin is a different decision -- see the header of the file.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMAGE=${1:-coco-platform:jazzy}
read -r -a D <<< "${DOCKER:-docker}"
hard() { grep -E "^$1==" "$HERE/pip-constraints.txt" | head -1; }
PINS="$(hard torch) $(hard stable_baselines3) $(hard gymnasium) $(hard cloudpickle) $(hard mujoco) $(hard tornado)"
"${D[@]}" run --rm -i -u 0 -e PINS="$PINS" --entrypoint bash "$IMAGE" -s <<'EOF'
set -e
site=$(python3 -c 'import sys; print("/usr/local/lib/python%d.%d/dist-packages" % sys.version_info[:2])')
rm -rf "${site:?}"/*
torch=$(echo "$PINS" | tr ' ' '\n' | grep '^torch==')
rest=$(echo "$PINS" | tr ' ' '\n' | grep -v '^torch==' | tr '\n' ' ')
pip3 install --break-system-packages --quiet --no-cache-dir \
  --index-url https://download.pytorch.org/whl/cpu "$torch" >&2
pip3 install --break-system-packages --quiet --no-cache-dir $rest >&2
python3 - "$site" <<'PY'
import importlib.metadata as md, sys
for d in sorted(md.distributions(path=[sys.argv[1]]),
                key=lambda d: d.metadata['Name'].lower()):
    print(f"{d.metadata['Name']}=={d.version}")
PY
EOF
