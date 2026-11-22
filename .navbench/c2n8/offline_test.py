#!/usr/bin/env python3
"""C2-NAV.8 offline test of nav_bench's two default-off overrides.

Runs before anything launches. Asserts, for `--leg-timeout` (new this
session) exactly what C2-NAV.7 asserted for `--goal`:

  1. NO-OVERRIDE IDENTITY. Absent the flag every leg gets `--timeout`,
     so every C2-NAV.0 ... C2-NAV.7 command reproduces unchanged.
  2. ISOLATION. One override moves one leg and leaves the other six at
     the default.
  3. REJECTION. Malformed, non-numeric, non-positive and unknown-scenario
     specs are refused by returning None -- BEFORE a simulator starts,
     not by a traceback three minutes into a run.

and re-asserts (1)-(3) for `--goal`, because C2-NAV.8 uses it and a
regression in it would silently attribute the shifted-goal result to the
committed goal.

Deliberately importable without ROS: nav_bench imports rclpy at module
scope, so this loads the two pure functions by AST surgery rather than by
`import nav_bench`. That is the same reason coco_rl's training env is
kept rclpy-free -- an offline check that needs a live graph is not an
offline check.

Run from THIS directory (C2-NAV.7's shared-scratch trap: a stray
numbers.py or trace.py alongside the script shadows the stdlib, breaks
numpy inside the import chain, and can print another run's output into
this one).
"""
import ast
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
BENCH = os.path.normpath(os.path.join(
    HERE, '..', '..', 'gazebo_models', 'scripts', 'nav_bench.py'))

WANT = ('TOUR', 'apply_goal_overrides', 'apply_leg_timeouts')


def load():
    """The three names, with nav_bench's rclpy imports left on the floor."""
    with open(BENCH) as f:
        tree = ast.parse(f.read(), BENCH)
    keep = [n for n in tree.body
            if (isinstance(n, ast.FunctionDef) and n.name in WANT)
            or (isinstance(n, ast.Assign)
                and any(getattr(t, 'id', None) in WANT for t in n.targets))]
    got = {getattr(n, 'name', None) or n.targets[0].id for n in keep}
    missing = set(WANT) - got
    if missing:
        print(f'FAIL: nav_bench.py has no {sorted(missing)}')
        sys.exit(1)
    ns = {}
    exec(compile(ast.Module(body=keep, type_ignores=[]), BENCH, 'exec'), ns)
    return ns


FAILED = []


def check(label, cond):
    print(f'{"ok  " if cond else "FAIL"}  {label}')
    if not cond:
        FAILED.append(label)


def main():
    ns = load()
    TOUR = ns['TOUR']
    goals = ns['apply_goal_overrides']
    caps = ns['apply_leg_timeouts']
    names = [t[0] for t in TOUR]

    print(f'-- TOUR: {len(TOUR)} legs, {names}')
    check('TOUR is the committed seven legs', len(TOUR) == 7)
    check('enclosure_entry goal is still the committed (-3.45, 2.95)',
          [t[1:3] for t in TOUR if t[0] == 'enclosure_entry'] ==
          [(-3.45, 2.95)])

    print('-- (1) no-override identity')
    check('--goal absent leaves TOUR byte-identical',
          goals(TOUR, None) == list(TOUR) and goals(TOUR, []) == list(TOUR))
    for d in (75.0, 150.0):
        got = caps(TOUR, d, None)
        check(f'--leg-timeout absent gives every leg {d} s',
              got == {n: d for n in names})
    check('--leg-timeout empty list is the same as absent',
          caps(TOUR, 75.0, []) == caps(TOUR, 75.0, None))

    print('-- (2) isolation')
    moved = goals(TOUR, ['enclosure_entry:-3.575,2.95'])
    check('--goal moves exactly the named leg',
          [(t[0], t[1], t[2]) for t in moved] ==
          [(t[0], -3.575 if t[0] == 'enclosure_entry' else t[1],
            2.95 if t[0] == 'enclosure_entry' else t[2]) for t in TOUR])
    check('--goal marks the moved leg in its probe string',
          all(('[GOAL OVERRIDDEN]' in t[3]) == (t[0] == 'enclosure_entry')
              for t in moved))
    ct = caps(TOUR, 75.0, ['enclosure_entry:200'])
    check('--leg-timeout moves exactly the named leg',
          ct == {n: (200.0 if n == 'enclosure_entry' else 75.0)
                 for n in names})
    ct2 = caps(TOUR, 75.0, ['enclosure_entry:200', 'enclosure_exit:120'])
    check('--leg-timeout is repeatable',
          ct2['enclosure_entry'] == 200.0 and ct2['enclosure_exit'] == 120.0
          and ct2['open_space'] == 75.0)
    check('--leg-timeout does not mutate TOUR',
          [t[1:3] for t in TOUR if t[0] == 'enclosure_entry'] ==
          [(-3.45, 2.95)])

    print('-- (3) rejection, before anything launches')
    for bad in ('enclosure_entry', 'enclosure_entry:', 'enclosure_entry:abc',
                'enclosure_entry:0', 'enclosure_entry:-5',
                'no_such_leg:200', ':200'):
        check(f'--leg-timeout rejects {bad!r}',
              caps(TOUR, 75.0, [bad]) is None)
    for bad in ('enclosure_entry', 'enclosure_entry:1',
                'enclosure_entry:a,b', 'no_such_leg:1,2'):
        check(f'--goal rejects {bad!r}', goals(TOUR, [bad]) is None)

    print('-- the C2-NAV.8 command line, resolved')
    t8 = goals(TOUR, ['enclosure_entry:-3.575,2.95'])
    c8 = caps(TOUR, 75.0, ['enclosure_entry:200'])
    for (n, x, y, _) in t8:
        print(f'   {n:16s} goal ({x:7.3f}, {y:6.3f})  cap {c8[n]:6.1f} s')

    print(f'\n{len(FAILED)} failed')
    return 1 if FAILED else 0


if __name__ == '__main__':
    sys.exit(main())
