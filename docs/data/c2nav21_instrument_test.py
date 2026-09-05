#!/usr/bin/env python3
"""C2-NAV.21 -- prove the new nav_bench degeneracy instrument before a
simulator is spent on it.

`nav_bench.py::_eval_cb` gained four things C2-NAV.20 could not measure:
the COMPLETE vs SHORT-CIRCUITED split, the zero-vx/forward margin, the
exact-tie count and the rotation-block span, plus the per-cycle illegal
split keyed by the throwing critic.

Three of those have a success condition of the form "we counted N", and
one -- the Oscillation illegal count -- has a success condition of the
form "we saw none". This repo has already paid for that shape once:

    Any check whose success condition is "we saw nothing" must first
    prove it can see something.

So this drives `_eval_cb` with synthetic `/evaluation` messages whose
answers are known by construction, including one that carries a latched
Oscillation ban, and refuses to pass unless the instrument reports it.

No ROS graph, no simulator: `_eval_cb` only touches `self.now()`,
`self._n_critics`, `self._lock` and `self.evals`, so the node is built
with `__new__` and those four are supplied directly.

    python3 -P docs/data/c2nav21_instrument_test.py
"""

import os
import sys
import threading

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPTS = os.path.join(HERE, '..', '..', 'gazebo_models', 'scripts')
sys.path.insert(0, os.path.abspath(SCRIPTS))

import nav_bench                                                 # noqa: E402

RES_SCALE_GD = 0.05 * 0.5 * 24.0        # 0.6
RES_SCALE_PD = 0.05 * 0.5 * 32.0        # 0.8
CRITICS = ['RotateToGoal', 'Oscillation', 'BaseObstacle', 'GoalAlign',
           'PathAlign', 'PathDist', 'GoalDist']


class CS:
    def __init__(self, name, raw, scale):
        self.name, self.raw_score, self.scale = name, raw, scale


class Vel:
    def __init__(self, x, th):
        self.x, self.y, self.theta = x, 0.0, th


class Traj:
    def __init__(self, vx, wz):
        self.velocity = Vel(vx, wz)


class Score:
    def __init__(self, vx, wz, total, n_critics):
        self.traj = Traj(vx, wz)
        self.total = total
        self.scores = [CS(CRITICS[i], 1.0, 1.0) for i in range(n_critics)]


class Illegal(Score):
    def __init__(self, vx, wz, critic):
        Score.__init__(self, vx, wz, -1.0, 0)
        self.scores = [CS(critic, -1.0, 0.0)]


class Eval:
    def __init__(self, twists, best_index):
        self.twists, self.best_index = twists, best_index


def make_node():
    n = nav_bench.NavBench.__new__(nav_bench.NavBench)
    n._ncrit = 0
    n._lock = threading.Lock()
    n.evals = nav_bench.Series()
    n.now = lambda: (0.0, 0.0)
    return n


def last(n):
    return n.evals.v[-1]


CHECKS = []


def chk(label, got, want):
    ok = got == want
    CHECKS.append(ok)
    print(f'  [{"OK " if ok else "FAIL"}] {label}: got {got!r} want {want!r}')
    return ok


def main():
    print('=' * 72)
    print('C2-NAV.21 instrument self-test -- nav_bench._eval_cb')
    print('=' * 72)

    # ---- 1. a clean cycle: 3 zero-vx, 3 forward, all complete --------
    n = make_node()
    tw = [Score(0.0, -1.0, 47.6, 7), Score(0.0, 0.0, 46.0, 7),
          Score(0.0, 1.0, 49.4, 7),
          Score(0.1, -1.0, 48.2, 7), Score(0.2, 0.0, 46.0, 7),
          Score(0.3, 1.0, 51.0, 7)]
    n._eval_cb(Eval(tw, 1))
    d = last(n)['degen']
    chk('critic width learned from the message', n._n_critics, 7)
    chk('n_complete', d['n_complete'], 6)
    chk('zero block counted', d['n_zero_complete'], 3)
    chk('forward block counted', d['n_fwd_complete'], 3)
    chk('best zero total', d['zero_best'], 46.0)
    chk('best forward total', d['fwd_best'], 46.0)
    chk('margin is an exact tie', d['margin'], 0.0)
    chk('rotation-block span', round(d['rot_span'], 6), 3.4)
    chk('trajectories at the minimum', d['n_at_min'], 2)
    chk('chosen wz recorded', last(n)['best']['wz'], 0.0)

    # ---- 2. forward genuinely better ---------------------------------
    n = make_node()
    tw = [Score(0.0, 0.0, 46.0, 7), Score(0.2, 0.0, 44.6, 7)]
    n._eval_cb(Eval(tw, 1))
    d = last(n)['degen']
    chk('margin positive when forward wins', round(d['margin'], 6), 1.4)
    chk('single minimum', d['n_at_min'], 1)

    # ---- 3. SHORT-CIRCUITED must not be compared to COMPLETE ---------
    #  A partial total is a partial SUM: it is smaller than the complete
    #  one would be, so counting it would invent a forward "win".
    n = make_node()
    tw = [Score(0.0, 0.0, 46.0, 7),
          Score(0.3, 0.0, 12.0, 3),          # broke out after 3 critics
          Score(0.1, 0.0, 47.2, 7)]
    n._eval_cb(Eval(tw, 0))
    d = last(n)['degen']
    chk('short-circuited excluded from n_complete', d['n_complete'], 2)
    chk('short-circuited cannot fake a forward win',
        round(d['margin'], 6), -1.2)
    chk('short-circuited cannot fake the minimum', d['min_total'], 46.0)

    # ---- 4. the "we saw nothing" check, proven able to see something -
    #  A latched Oscillation ban makes every trajectory of one rotation
    #  sign illegal. If the instrument cannot report that, a run in which
    #  Oscillation reports 0 is uninterpretable.
    n = make_node()
    tw = ([Illegal(0.0, w, 'Oscillation') for w in (0.5, 1.0)]
          + [Illegal(0.3, 0.5, 'Oscillation')]
          + [Illegal(0.3, -1.0, 'BaseObstacle')]
          + [Score(0.0, -0.5, 46.0, 7), Score(0.1, -0.5, 46.6, 7)])
    n._eval_cb(Eval(tw, 4))
    e = last(n)
    chk('illegal total', e['n_illegal'], 4)
    chk('Oscillation illegals SEEN', e['illegal'].get('Oscillation'), 3)
    chk('BaseObstacle illegals seen', e['illegal'].get('BaseObstacle'), 1)
    chk('degeneracy computed from the legal remainder only',
        e['degen']['n_complete'], 2)
    chk('margin over the surviving half', round(e['degen']['margin'], 6),
        -0.6)

    # ---- 5. and the same instrument reports a genuine zero ------------
    n = make_node()
    n._eval_cb(Eval([Score(0.0, 0.0, 46.0, 7), Score(0.2, 0.0, 45.0, 7)], 1))
    e = last(n)
    chk('no Oscillation illegals when there are none',
        e['illegal'].get('Oscillation', 0), 0)
    chk('but the cycle is still fully populated',
        (e['degen']['n_complete'], e['n_illegal']), (2, 0))

    # ---- 6. an all-illegal cycle must not crash or invent a margin ----
    n = make_node()
    n._eval_cb(Eval([Illegal(0.0, 0.0, 'BaseObstacle')], -1))
    d = last(n)['degen']
    chk('no complete trajectories', d['n_complete'], 0)
    chk('margin is None, not 0', d['margin'], None)
    chk('rot_span is None, not 0', d['rot_span'], None)
    chk('n_at_min is None, not 0', d['n_at_min'], None)
    chk('best is None when best_index is -1', last(n)['best'], None)

    # ---- 7. a zero-only cycle (no legal forward) ----------------------
    n = make_node()
    n._eval_cb(Eval([Score(0.0, 0.0, 46.0, 7),
                     Illegal(0.3, 0.0, 'BaseObstacle')], 0))
    d = last(n)['degen']
    chk('margin None when no forward trajectory survives',
        d['margin'], None)
    chk('zero block still measured', d['zero_best'], 46.0)

    print()
    ok = all(CHECKS)
    print(f'  {sum(CHECKS)}/{len(CHECKS)} checks passed -- '
          + ('INSTRUMENT VALIDATED' if ok else 'INSTRUMENT NOT VALIDATED'))
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
