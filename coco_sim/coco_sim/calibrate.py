# Copyright 2026 Gautham Anil
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Fit the MuJoCo contact parameters to the measured Gazebo yaw response.

This is the harness that produced ``WHEEL_FRICTION``, ``CONTACT_SOLREF``
and ``CONTACT_SOLIMP_D0`` in ``coco_sim.mjcf``. It lives in the package,
under test, for one reason: **a fitted constant whose harness lives in a
scratch directory is not reproducible, and this repo does not ship
measured artefacts with no reproduction path.**

Why it is written this way
--------------------------
The previous harness swept parameters by STRING-REPLACING literals in the
generated MJCF — patching ``solref="0.1 1"``. When the fitted value 0.25
was written back into ``mjcf.py``, that pattern stopped matching and the
``solref`` lever silently detached: three distinct values produced
bit-for-bit identical scores while the sweep printed a normal-looking
table. The fit that the world currently ships was made through a harness
in that state.

So there is no string replacement here. ``build_mjcf`` takes the contact
parameters as arguments, and :func:`audit_levers` asserts every one of
them still moves the score before any fit is trusted. That check is not
optional politeness — it is the only thing standing between a sweep and a
plausible-looking table of identical numbers.

The reference
-------------
``coco_sim/reference/yaw_gazebo_baseline.csv``, magnitude-averaged. Both
signs are measured because Gazebo disagrees with its own mirrored command
by 1.36x at 2.5 rad, so the average is the honest target and the top of
the range is known to be soft.
"""

import csv
import math
import os

from coco_config.robot import (WHEEL_RADIUS, WHEEL_SEPARATION,
                               WHEEL_SEPARATION_MULTIPLIER)

from coco_sim.mjcf import build_mjcf
from coco_sim.sweep import assert_lever_is_connected

import mujoco

MAX_LIN = 0.4
ARC_S = 5.0
SETTLE_S = 2.0

REFERENCE = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'reference', 'yaw_gazebo_baseline.csv')

# The four commands the score is taken over: two inside the range the lane
# hold actually issues, two well outside it. Scoring only small commands
# would fit the easy end; scoring only large ones would fit the end where
# Gazebo is not repeatable against itself.
SCORE_COMMANDS = (0.05, 0.25, 1.00, 2.50)


def gazebo_reference(path=None):
    """{|cmd_yaw|: efficiency %} from the measured Gazebo baseline.

    Magnitude-averaged over the +/- pair at each command, and computed
    from the CSV rather than transcribed — a hard-coded copy of a
    measurement is a second source of truth waiting to drift.
    """
    sums, counts = {}, {}
    with open(path or REFERENCE) as handle:
        for row in csv.DictReader(handle):
            cmd = abs(round(float(row['cmd_yaw']), 4))
            eff = abs(float(row['achieved'])) / abs(float(row['cmd_yaw']))
            sums[cmd] = sums.get(cmd, 0.0) + 100.0 * eff
            counts[cmd] = counts.get(cmd, 0) + 1
    return {k: sums[k] / counts[k] for k in sums}


def achieved_yaw(rate, sep_mult=WHEEL_SEPARATION_MULTIPLIER, **model_kw):
    """Yaw turned over a 5 s arc at `rate` rad/s, after settling."""
    model = mujoco.MjModel.from_xml_string(build_mjcf(**model_kw))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    sub = int(round(0.1 / model.opt.timestep))
    sep = WHEEL_SEPARATION * sep_mult

    def drive(ang, seconds):
        left = (MAX_LIN - ang * sep / 2.0) / WHEEL_RADIUS
        right = (MAX_LIN + ang * sep / 2.0) / WHEEL_RADIUS
        for _ in range(int(seconds / 0.1)):
            data.ctrl[2] = data.ctrl[3] = left
            data.ctrl[0] = data.ctrl[1] = right
            for _ in range(sub):
                mujoco.mj_step(model, data)

    def yaw():
        w, x, y, z = data.qpos[3:7]
        return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

    drive(0.0, SETTLE_S)
    start = yaw()
    drive(rate, ARC_S)
    return yaw() - start


def efficiency(cmd, **kw):
    """Achieved yaw as a percentage of the commanded arc."""
    return 100.0 * achieved_yaw(cmd / ARC_S, **kw) / cmd


def score(friction=None, solref=None, solimp=None,
          sep_mult=WHEEL_SEPARATION_MULTIPLIER, commands=SCORE_COMMANDS,
          reference=None):
    """Worst |ratio - 1| against Gazebo over `commands`. Lower is better.

    Returns ``(worst, {cmd: (mujoco_pct, ratio)})``. A ratio is always
    taken as ``max(r, 1/r)`` so that under- and over-steering are
    penalised alike; a plain difference would prefer a model that
    over-rotates.
    """
    ref = reference if reference is not None else gazebo_reference()
    worst, detail = 0.0, {}
    for cmd in commands:
        pct = efficiency(cmd, friction=friction, solref=solref,
                         solimp=solimp, sep_mult=sep_mult)
        if pct <= 1e-6:
            # the model failed to turn at all, or turned the wrong way
            return 99.0, {cmd: (pct, float('inf'))}
        ratio = ref[cmd] / pct
        detail[cmd] = (pct, ratio)
        worst = max(worst, max(ratio, 1.0 / ratio))
    return worst, detail


LEVERS = {
    'friction': (0.25, 0.40, 0.70),
    'solref': (0.05, 0.25, 0.50),
    'solimp': (0.20, 0.50, 0.90),
    'sep_mult': (1.00, 1.10, 1.25),
}


def audit_levers(levers=None, commands=(0.25, 1.00)):
    """Assert every swept parameter still reaches the model.

    Run this BEFORE trusting any fit. The failure it catches does not look
    like a failure: the sweep completes, the table prints, and every row
    holds the same number.

    Returns ``{name: spread}``; raises ``DisconnectedParameter`` on the
    first lever that does not move the score.
    """
    ref = gazebo_reference()
    spreads = {}
    for name, values in (levers or LEVERS).items():
        results = [
            (v, score(**{name: v}, commands=commands, reference=ref)[0])
            for v in values]
        spreads[name] = assert_lever_is_connected(name, results)
    return spreads


def main():
    """Audit the levers, then grid-search the contact parameters."""
    import argparse
    import itertools
    ap = argparse.ArgumentParser(description='Fit MuJoCo contact to Gazebo.')
    ap.add_argument('--skip-audit', action='store_true')
    args = ap.parse_args()

    if not args.skip_audit:
        print('lever audit (spread of worst-ratio across the sweep):')
        for name, spread in audit_levers().items():
            print(f'  {name:<10} live, spread {spread:.4f}')
        print()

    ref = gazebo_reference()
    results = []
    print(f"{'friction':>9}{'solref':>8}{'solimp':>8}{'sep':>6}{'worst':>9}")
    for friction, solref, solimp, sep in itertools.product(
            (0.25, 0.30, 0.40, 0.55, 0.70),
            (0.05, 0.10, 0.25, 0.40),
            (0.20, 0.50, 0.90),
            (1.10,)):
        worst, _ = score(friction, solref, solimp, sep, reference=ref)
        results.append((worst, (friction, solref, solimp, sep)))
        print(f'{friction:>9}{solref:>8}{solimp:>8}{sep:>6}{worst:>9.3f}')
    results.sort()
    worst, best = results[0]
    print(f'\nBEST worst={worst:.3f} at friction={best[0]} solref={best[1]} '
          f'solimp={best[2]} sep={best[3]}')
    _, detail = score(*best, reference=ref)
    for cmd, (pct, ratio) in sorted(detail.items()):
        print(f'  |cmd| {cmd:5.2f}: gz {ref[cmd]:5.1f}%  mj {pct:5.1f}%  '
              f'ratio {ratio:.3f}')


if __name__ == '__main__':
    main()
