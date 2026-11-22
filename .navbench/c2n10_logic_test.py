import re

src = open("gazebo_models/scripts/nav_bench.py").read()


def extract(name):
    m = re.search(rf"\ndef {name}\(.*?\n(?=\ndef |\Z)", src, re.S)
    return m.group(0)


code = "TOUR = " + re.search(r"\nTOUR = \[.*?\n\]\n", src, re.S).group(0).split('=', 1)[1]
code += extract('apply_goal_overrides')
code += extract('apply_waypoint_insert')
code += extract('apply_leg_timeouts')
g = {}
exec(code, g)
TOUR = g['TOUR']
apply_goal_overrides = g['apply_goal_overrides']
apply_waypoint_insert = g['apply_waypoint_insert']
apply_leg_timeouts = g['apply_leg_timeouts']


def simulate_main(goal_specs, only_spec, waypoint_specs, timeout_specs=None):
    """Reproduce main()'s exact post-fix ordering: goal overrides, THEN
    --only filter (by original name), THEN waypoint insert, THEN
    timeouts -- and return the final executed leg-name sequence."""
    tour = apply_goal_overrides(TOUR, goal_specs)
    assert tour is not None
    if only_spec:
        want = set(only_spec.split(','))
        tour = [t for t in tour if t[0] in want]
    tour = apply_waypoint_insert(tour, waypoint_specs)
    assert tour is not None
    timeouts = apply_leg_timeouts(tour, 75.0, timeout_specs)
    assert timeouts is not None
    return [t[0] for t in tour], timeouts


# --- the exact C2-NAV.10 invocation that failed live ---
names, timeouts = simulate_main(
    ['enclosure_entry:-3.575,2.95'],
    'corridor_gate,enclosure_entry',
    ['enclosure_entry:-3.40,1.35'],
    ['enclosure_entry:200'])
print("C2-NAV.10 --only + --waypoint executed sequence:", names)
assert names == ['corridor_gate', 'enclosure_entry_waypoint', 'enclosure_entry'], \
    f"REGRESSION: waypoint leg dropped by --only filter, got {names}"
print("PASS: waypoint leg survives --only filtering (the bug is fixed)")
assert timeouts['enclosure_entry_waypoint'] == 75.0
assert timeouts['enclosure_entry'] == 200.0
print("PASS: timeouts correct for both the inserted leg and the target leg")

# --- full tour, no --only, --waypoint set: unchanged shape plus insert ---
names2, _ = simulate_main(None, None, ['enclosure_entry:-3.40,1.35'], None)
assert names2 == ['open_space', 'wall_adjacent', 'wall_parallel', 'obstacle_corner',
                   'corridor_gate', 'enclosure_entry_waypoint', 'enclosure_entry',
                   'enclosure_exit']
print("PASS: full-tour insertion unaffected by the reorder:", names2)

# --- no --waypoint at all: every prior C2-NAV.0..C2-NAV.9 command must
# reproduce byte-identically regardless of --only ordering ---
names3, _ = simulate_main(['enclosure_entry:-3.575,2.95'], 'corridor_gate,enclosure_entry',
                           None, ['enclosure_entry:200'])
assert names3 == ['corridor_gate', 'enclosure_entry']
print("PASS: --only without --waypoint (C2-NAV.8's own invocation) unaffected:", names3)

names4, _ = simulate_main(None, None, None, None)
assert names4 == [t[0] for t in TOUR]
print("PASS: no flags at all reproduces TOUR unchanged:", names4)

print()
print("ALL REGRESSION TESTS PASS")
