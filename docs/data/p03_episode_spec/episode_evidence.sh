#!/usr/bin/env bash
# Produce the determinism / variation / validity evidence for the report.
REPO="/home/gautham/ros2_ws(personal)/src/coco-robot-ros2/.claude/worktrees/p02-browser"
export COCO_WS="$HOME/coco_ws_build"
# shellcheck disable=SC1091
source "$REPO/setup_env.sh" >/dev/null 2>&1
cd "$REPO/coco_sim" || exit 9
python3 - <<'EOF'
import hashlib
from coco_sim.episode import (generate_episode, target_x_bounds,
                              target_y_bounds, LEVELS)


def h(spec):
    return hashlib.sha256(spec.to_json().encode()).hexdigest()[:16]


print('## same seed, generated twice (sha256 of the JSON manifest)')
for level in LEVELS:
    for seed in (0, 1827):
        a = generate_episode(seed=seed, level=level)
        b = generate_episode(seed=seed, level=level)
        print(f'{level:9s} seed={seed:<5d} {h(a)} {h(b)} '
              f'identical={a.to_json() == b.to_json()}')

print()
print('## different seeds, level=positions')
for seed in (0, 1, 2):
    s = generate_episode(seed=seed, level='positions')
    row = '  '.join(f'{t.colour}@({t.x:.4f},{t.y:+.4f})' for t in s.targets)
    print(f'seed={seed} ask={s.requested_colour:6s} {s.episode_id}  {row}')

print()
print('## different seeds, level=colours (lane order, y ascending)')
for seed in (0, 1, 2):
    s = generate_episode(seed=seed, level='colours')
    order = [t.colour for t in sorted(s.targets, key=lambda t: t.y)]
    print(f'seed={seed} {order}')

print()
print('## envelope (derived from coco_config)')
for d in (0.020, 0.032):
    near, far = target_x_bounds(d)
    print(f'x for diameter {d*1000:.0f} mm: [{near:.4f}, {far:.4f}]')
lo, hi = target_y_bounds()
print(f'y: [{lo:+.4f}, {hi:+.4f}]')

print()
print('## validity sweep')
for level in LEVELS:
    n = 0
    for seed in range(10000):
        generate_episode(seed=seed, level=level)   # validates or raises
        n += 1
    print(f'{level:9s} {n} seeds generated, all passed validate_episode')

print()
print('## task view (what the robot may see) for seed 1827, positions')
print(generate_episode(seed=1827, level='positions').task_view())
EOF
