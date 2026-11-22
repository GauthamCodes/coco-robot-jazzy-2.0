import re

src = open("gazebo_models/scripts/nav_bench.py").read()


def extract(name):
    m = re.search(rf"\ndef {name}\(.*?\n(?=\ndef |\Z)", src, re.S)
    return m.group(0)


code = "TOUR = " + re.search(r"\nTOUR = \[.*?\n\]\n", src, re.S).group(0).split('=', 1)[1]
code += extract('apply_goal_overrides')
code += extract('apply_waypoint_insert')
code += extract('apply_through_poses')
code += extract('apply_leg_timeouts')
g = {}
exec(code, g)
TOUR = g['TOUR']
apply_goal_overrides = g['apply_goal_overrides']
apply_waypoint_insert = g['apply_waypoint_insert']
apply_through_poses = g['apply_through_poses']
apply_leg_timeouts = g['apply_leg_timeouts']


def simulate_main(goal_specs, only_spec, waypoint_specs, through_specs,
                   timeout_specs=None):
    """Reproduce main()'s exact ordering: goal overrides, --only filter
    (by original name), waypoint insert, THEN through-poses (consulted by
    name, cannot be shifted by either of the first two), THEN timeouts."""
    tour = apply_goal_overrides(TOUR, goal_specs)
    assert tour is not None
    if only_spec:
        want = set(only_spec.split(','))
        tour = [t for t in tour if t[0] in want]
    tour = apply_waypoint_insert(tour, waypoint_specs)
    assert tour is not None
    through = apply_through_poses(tour, through_specs)
    assert through is not None
    timeouts = apply_leg_timeouts(tour, 75.0, timeout_specs)
    assert timeouts is not None
    return [t[0] for t in tour], through, timeouts


# --- the exact C2-NAV.11 invocation: --only + --through-pose, no --waypoint ---
names, through, timeouts = simulate_main(
    ['enclosure_entry:-3.575,2.95'],
    'corridor_gate,enclosure_entry',
    None,
    ['enclosure_entry:-3.40,1.35'],
    ['enclosure_entry:200'])
print("C2-NAV.11 --only + --through-pose executed sequence:", names)
assert names == ['corridor_gate', 'enclosure_entry'], \
    f"REGRESSION: through-pose must NOT insert a leg, got {names}"
print("PASS: --through-pose does not change the leg sequence "
      "(unlike --waypoint)")
assert through == {'corridor_gate': [], 'enclosure_entry': [(-3.4, 1.35)]}
print("PASS: through-pose correctly attached to enclosure_entry only:",
      through)
assert timeouts['enclosure_entry'] == 200.0
print("PASS: leg timeout applies to the single enclosure_entry leg "
      "(no separate waypoint leg to time)")

# --- repeatable --through-pose for the same scenario: order preserved ---
names2, through2, _ = simulate_main(
    None, 'enclosure_entry', None,
    ['enclosure_entry:-3.6,1.0', 'enclosure_entry:-3.4,1.35'], None)
assert through2['enclosure_entry'] == [(-3.6, 1.0), (-3.4, 1.35)]
print("PASS: repeated --through-pose for one scenario preserves order:",
      through2['enclosure_entry'])

# --- malformed / unknown specs are rejected before anything launches ---
assert apply_through_poses(TOUR, ['enclosure_entry-3.4,1.35']) is None
print("PASS: malformed spec (no ':') rejected")
assert apply_through_poses(TOUR, ['enclosure_entry:not,a,number']) is None
print("PASS: malformed spec (wrong arity) rejected")
assert apply_through_poses(TOUR, ['enclosure_entry:x,1.35']) is None
print("PASS: non-numeric spec rejected")
assert apply_through_poses(TOUR, ['no_such_leg:-3.4,1.35']) is None
print("PASS: unknown scenario name rejected")

# --- no --through-pose at all: every prior C2-NAV.0..C2-NAV.10 command
# must reproduce byte-identically (empty-list mapping, no dispatch change) ---
names3, through3, _ = simulate_main(None, None, None, None, None)
assert names3 == [t[0] for t in TOUR]
assert through3 == {t[0]: [] for t in TOUR}
print("PASS: no flags at all reproduces TOUR unchanged, "
      "through-poses all empty:", names3)

# --- --waypoint and --through-pose are independent mechanisms and do not
# collide when both happen to be passed (not used together in C2-NAV.11,
# but the two must not silently corrupt each other) ---
names4, through4, _ = simulate_main(
    None, None, ['enclosure_entry:-3.4,1.35'],
    ['corridor_gate:-2.6,1.0'], None)
assert names4 == ['open_space', 'wall_adjacent', 'wall_parallel',
                  'obstacle_corner', 'corridor_gate',
                  'enclosure_entry_waypoint', 'enclosure_entry',
                  'enclosure_exit']
assert through4['corridor_gate'] == [(-2.6, 1.0)]
assert through4['enclosure_entry_waypoint'] == []
print("PASS: --waypoint and --through-pose coexist without interference:",
      names4, through4['corridor_gate'])

print()
print("ALL REGRESSION TESTS PASS")
