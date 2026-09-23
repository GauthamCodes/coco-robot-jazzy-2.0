#!/usr/bin/env bash
# How many generated episodes have a target standing in another's approach
# corridor? Measured against the generator as committed (f971078).
REPO="/home/gautham/ros2_ws(personal)/src/coco-robot-ros2/.claude/worktrees/p02-browser"
export COCO_WS="$HOME/coco_ws_build"
# shellcheck disable=SC1091
source "$REPO/setup_env.sh" >/dev/null 2>&1
cd "$REPO/coco_sim" || exit 9
python3 - <<'EOF'
from coco_sim.episode import HALF_FOOTPRINT_Y, LEVELS, generate_episode


def blocked(spec):
    """(far, near) pairs where near stands in far's straight approach."""
    out = []
    for far in spec.targets:
        for near in spec.targets:
            if near is far or near.x >= far.x:
                continue
            if abs(near.y - far.y) < HALF_FOOTPRINT_Y + near.radius:
                out.append((far.colour, near.colour))
    return out


for level in LEVELS:
    bad = any_req = 0
    for seed in range(10000):
        spec = generate_episode(seed=seed, level=level)
        b = blocked(spec)
        if b:
            bad += 1
            if any(f == spec.requested_colour for f, _ in b):
                any_req += 1
    print(f'{level:9s} episodes with a blocked corridor: {bad}/10000; '
          f'blocking the REQUESTED target: {any_req}/10000')
print(f'HALF_FOOTPRINT_Y = {HALF_FOOTPRINT_Y:.4f} m')
EOF
