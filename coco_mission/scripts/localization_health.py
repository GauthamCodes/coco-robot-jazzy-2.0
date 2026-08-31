#!/usr/bin/env python3
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
localization_health — the pure core of the C2-M5 localization signal.

**Nothing runs this yet, and that is deliberate.** No node imports it, no
launch file starts it, and it takes no recovery action of any kind.
C2-M5.0's job was to find out what the robot can actually observe about
its own localization; this file is that answer written down in a form
C2-M5.1 can wire up and the tests can pin. Same split as
``mission_states.py`` (pure) and ``mission_executive.py`` (the adapter),
for the same reason: a rule you cannot test without Gazebo is a rule
nobody tests.

What C2-M5.0 measured, and why this file is shaped the way it is
----------------------------------------------------------------
**Five runs**, fresh simulator each, never ``--fast``: two legs that
finished, one that failed with no injection at all, and two injected
divergences. Full numbers in ``RESULTS.md``, "C2-M5.0 localization
health". Three findings decide this design.

1. **AMCL's covariance does not detect a divergence, and at the moment
   of one it moves the WRONG WAY.** On the injected 3 m divergence,
   ``sigma_xy`` fell to **0.070 m** — its smallest value of the whole
   leg, and below the smallest value in the entire healthy run (0.248 m)
   — at the instant the pose became 3 m wrong. It took **24.5 s** to
   climb past the healthy maximum. A rule of the form "covariance above
   X means lost" would have called that robot healthier than usual for
   the first half-minute of being lost.

   Part of that dip is imposed: the injection hands AMCL a small
   covariance on purpose, because that is what a filter collapsing onto
   a wrong mode looks like. What is *not* imposed, and what is measured,
   is the 24.5 s AMCL then took to notice — 13.9 s on the second
   divergence run. And on the same stretch of ground, ``diverged2``'s
   median ``sigma_xy`` was **0.281 m against every healthy run's 0.37 to
   0.39**, while its position was 3.14 m wrong: not merely uninformative,
   but *better than healthy* while lost.

2. **The scan-vs-map likelihood does detect it, in 0.4 s.** Take the
   laser scan, place its endpoints in the map using the current
   map->laser transform, and measure how far each endpoint is from the
   nearest occupied cell. That is the likelihood field ``nav2_amcl``
   already scores particles against and never publishes. Healthy
   RETURN_HOME: mean endpoint distance 0.001-0.314 m, median 0.053.
   Diverged: it left that envelope **0.4 s** after the true error
   crossed 1 m, on **both** divergence runs, and stayed out for 62.6%
   and 91.5% of those legs.

So the primary input is consistency, not confidence. Covariance is
carried because it is free and may yet matter for something else; it is
**not** the divergence test.

3. **It does not separate the failure that nobody injected.** ``healthy2``
   failed on its own, with a position error of 0.300 m — inside the
   healthy band — and a heading error that reached 1.31 rad. Compared on
   the same ground, the scan signal orders it correctly but the gap
   between the worst leg that finished (0.211 m, 0.457 near) and this one
   (0.265 m, 0.339 near) is 0.054 m and 0.118. **Five runs cannot place a
   threshold in a gap that size, so this file does not contain one.**

The blind spot, stated rather than discovered later
---------------------------------------------------
``coco_world.pgm`` is a 2D slice of the flat world. The ramp and the
raised platform are **not in it**, so a scan taken up there disagrees
with the map for a reason that has nothing to do with localization. The
healthy run's own worst samples are exactly that: mean endpoint distance
0.29-0.31 m at world x around 6.7, still near the platform, with a true
error of only 0.26 m. ``Observation.on_mapped_ground`` gates it, and
without that gate the signal is wrong in precisely the place the mission
spends a third of its time.

Why there are no default thresholds
------------------------------------
Because five runs do not make a threshold, and this repo's standing
rule is that an unjustified number is worse than an admitted gap.
:class:`Thresholds` therefore has **no defaults** and must be
constructed explicitly; :func:`classify` returns ``UNKNOWN`` rather than
guessing when it has not been given one. :data:`C2M50_ENVELOPE` records
what was actually observed, as ranges, and is deliberately NOT a
``Thresholds`` — converting one into the other is C2-M5.1's job, after
it has run enough legs to know the healthy spread.

The one criterion that IS settled is freshness, because its bound is not
invented: it is the stack's own ``transform_tolerance``. AMCL post-dates
map->odom by that amount, so on a healthy stack the transform's "age" is
about **-0.5 s** — negative, because it is stamped in the future.
Measured healthy: -0.44 s. If that age climbs through zero and keeps
going, AMCL has stopped republishing and the pose being steered by is
whatever was last latched. No new number is being invented to say so.
"""

from dataclasses import dataclass

# Verdicts. Deliberately descriptive rather than a safety judgement:
# "the scan disagrees with the map" is a measurement, "the robot is lost"
# is a decision, and C2-M5.0 does not make decisions.
CONSISTENT = 'CONSISTENT'
INCONSISTENT = 'INCONSISTENT'
STALE = 'STALE'
UNKNOWN = 'UNKNOWN'

VERDICTS = (CONSISTENT, INCONSISTENT, STALE, UNKNOWN)

# Reasons, so a verdict says which check produced it.
NO_POSE = 'NO_POSE'
NO_SCAN_MATCH = 'NO_SCAN_MATCH'
OFF_MAPPED_GROUND = 'OFF_MAPPED_GROUND'
NO_THRESHOLDS = 'NO_THRESHOLDS'
POSE_STALE = 'POSE_STALE'
TRANSFORM_STALE = 'TRANSFORM_STALE'
SCAN_DISAGREES = 'SCAN_DISAGREES'
FEW_BEAMS = 'FEW_BEAMS'
OK = 'OK'

# nav2_params.yaml, amcl.transform_tolerance. AMCL stamps map->odom this
# far into the future, so a healthy map_odom_age sits near -0.5 s.
AMCL_TRANSFORM_TOLERANCE = 0.5


@dataclass(frozen=True)
class Observation:
    """Everything about its own localization the robot can actually see.

    Every field is computable on the robot from the map it was given,
    its own laser, its own TF tree and its own topics. **No field may
    come from the simulator.** ``c2m5_locrec.py`` records a ``gt_*``
    block beside these for offline scoring; none of it appears here, and
    that omission is the milestone's central constraint rather than an
    oversight.

    ``None`` means "not observed", which is different from zero and is
    why the fields are optional rather than defaulted.
    """

    # Scan against map, under the pose currently being published.
    lik_mean_d: float = None        # m, mean endpoint-to-obstacle distance
    lik_frac_near: float = None     # fraction of endpoints within LIK_NEAR_M
    lik_beams: int = 0              # endpoints that landed inside the map

    # AMCL's opinion of itself. Recorded, NOT the divergence test — see
    # the module docstring for the 24.5 s and the wrong-way dip.
    cov_sigma_xy: float = None      # m, sqrt(cov_xx + cov_yy)

    # Freshness. map_odom_age is normally NEGATIVE by roughly
    # AMCL_TRANSFORM_TOLERANCE; see the module docstring.
    amcl_age: float = None          # s, since the last /amcl_pose stamp
    map_odom_age: float = None      # s, now - stamp of map->odom

    # Discontinuity: how far map->odom moved in one sample. A large jump
    # is a relocalization EVENT, not by itself a wrongness measure — a
    # correct relocalization jumps too.
    map_odom_step: float = None     # m

    # The gate. False on the ramp and the platform, which are not in the
    # 2D map, and where lik_* is therefore uninterpretable.
    on_mapped_ground: bool = True


@dataclass(frozen=True)
class Thresholds:
    """Bounds for :func:`classify`. **No defaults, on purpose.**

    C2-M5.0 measured envelopes, not thresholds. Constructing this
    requires naming every number, so a threshold can only enter the
    system by someone typing it, with the run that justifies it in the
    commit message. :data:`C2M50_ENVELOPE` is what was observed.
    """

    lik_mean_d_max: float
    lik_frac_near_min: float
    min_beams: int = 10
    max_amcl_age: float = 5.0
    # Healthy is about -0.5 s. Zero means "AMCL has stopped republishing
    # the correction and the tolerance window it bought has run out",
    # which needs no new constant.
    max_map_odom_age: float = 0.0


@dataclass(frozen=True)
class Envelope:
    """A measured range. Explicitly NOT a threshold."""

    lo: float
    hi: float
    median: float
    n: int
    source: str


@dataclass(frozen=True)
class Verdict:
    """The answer, and which check produced it."""

    verdict: str
    reason: str
    detail: str = ''

    def __bool__(self):
        """True only for CONSISTENT.

        UNKNOWN is not good news and must not read as good news at a
        call site that writes ``if health:``.
        """
        return self.verdict == CONSISTENT


def classify(obs, thresholds=None):
    """Judge one observation. Pure: no clock, no ROS, no I/O.

    Order matters and is not arbitrary. Freshness comes first because a
    stale estimate makes every other field a statement about the past;
    the mapped-ground gate comes next because off the map the
    consistency metric is meaningless rather than bad; the consistency
    test comes last because it is the only one that needs a number
    nobody has yet justified.
    """
    if thresholds is None:
        return Verdict(UNKNOWN, NO_THRESHOLDS,
                       'no measured threshold exists yet; see C2M50_ENVELOPE')

    # ── freshness ────────────────────────────────────────────────────────
    if obs.amcl_age is None and obs.map_odom_age is None:
        return Verdict(UNKNOWN, NO_POSE, 'no pose and no map->odom observed')
    if obs.amcl_age is not None and obs.amcl_age > thresholds.max_amcl_age:
        return Verdict(STALE, POSE_STALE,
                       f'/amcl_pose is {obs.amcl_age:.2f} s old, over '
                       f'{thresholds.max_amcl_age:.2f}')
    if (obs.map_odom_age is not None
            and obs.map_odom_age > thresholds.max_map_odom_age):
        return Verdict(STALE, TRANSFORM_STALE,
                       f'map->odom age {obs.map_odom_age:+.2f} s has passed '
                       f'{thresholds.max_map_odom_age:+.2f}; AMCL normally '
                       f'post-dates it by {AMCL_TRANSFORM_TOLERANCE:.2f} s')

    # ── the gate ─────────────────────────────────────────────────────────
    if not obs.on_mapped_ground:
        return Verdict(UNKNOWN, OFF_MAPPED_GROUND,
                       'the ramp and the platform are not in the 2D map, so '
                       'a scan taken there disagrees with it for reasons '
                       'that are not localization')
    if obs.lik_mean_d is None or obs.lik_frac_near is None:
        return Verdict(UNKNOWN, NO_SCAN_MATCH, 'no scan-vs-map score')
    if obs.lik_beams < thresholds.min_beams:
        return Verdict(UNKNOWN, FEW_BEAMS,
                       f'{obs.lik_beams} endpoints inside the map, under '
                       f'{thresholds.min_beams}')

    # ── consistency ──────────────────────────────────────────────────────
    if (obs.lik_mean_d > thresholds.lik_mean_d_max
            or obs.lik_frac_near < thresholds.lik_frac_near_min):
        return Verdict(
            INCONSISTENT, SCAN_DISAGREES,
            f'mean endpoint distance {obs.lik_mean_d:.3f} m '
            f'(max {thresholds.lik_mean_d_max:.3f}), '
            f'{obs.lik_frac_near:.3f} of endpoints near an obstacle '
            f'(min {thresholds.lik_frac_near_min:.3f})')
    return Verdict(CONSISTENT, OK,
                   f'mean endpoint distance {obs.lik_mean_d:.3f} m, '
                   f'{obs.lik_frac_near:.3f} near')


# ── what C2-M5.0 actually observed ───────────────────────────────────────
# Ranges over the RETURN_HOME leg of each run, 10 Hz sampling. These are
# a RECORD, not a configuration: nothing reads this dict to make a
# decision, and turning it into `Thresholds` is C2-M5.1's job once it has
# more than one healthy leg to spread. Source runs are named so a future
# session can tell how thin the evidence is.
C2M50_ENVELOPE = {
    # The two legs that finished.
    'healthy1': {
        'lik_mean_d': Envelope(0.0008, 0.3139, 0.0533, 804, 'healthy1'),
        'lik_frac_near': Envelope(0.3167, 1.0000, 0.8751, 804, 'healthy1'),
        'cov_sigma_xy': Envelope(0.2482, 0.5683, 0.3763, 804, 'healthy1'),
        'err_xy_ground_truth': Envelope(0.0057, 0.5126, 0.2569, 804,
                                        'healthy1'),
    },
    'obstacle1': {
        'lik_mean_d': Envelope(0.0052, 0.3851, 0.0616, 510, 'obstacle1'),
        'lik_frac_near': Envelope(0.1622, 1.0000, 0.8833, 510, 'obstacle1'),
        'cov_sigma_xy': Envelope(0.2400, 1.0003, 0.4103, 510, 'obstacle1'),
        'err_xy_ground_truth': Envelope(0.0269, 0.5194, 0.1904, 510,
                                        'obstacle1'),
    },
    # The leg that failed with NO injection and a healthy position: the
    # class this signal does not separate.
    'healthy2': {
        'lik_mean_d': Envelope(0.0483, 0.3717, 0.2648, 119, 'healthy2'),
        'lik_frac_near': Envelope(0.1500, 1.0000, 0.3393, 119, 'healthy2'),
        'cov_sigma_xy': Envelope(0.2044, 0.5122, 0.3721, 119, 'healthy2'),
        'err_xy_ground_truth': Envelope(0.1372, 0.4472, 0.2999, 119,
                                        'healthy2'),
    },
    # Injected divergence, -3 m in y. diverged1 also carried a heading
    # error; diverged2 preserved the true heading, so it is the cleaner
    # position-only case.
    'diverged1': {
        'lik_mean_d': Envelope(0.0267, 0.5569, 0.3764, 1348, 'diverged1'),
        'lik_frac_near': Envelope(0.0536, 1.0000, 0.3200, 1348, 'diverged1'),
        'cov_sigma_xy': Envelope(0.0702, 1.2408, 0.5786, 1348, 'diverged1'),
        'err_xy_ground_truth': Envelope(0.2748, 4.2989, 2.8241, 1348,
                                        'diverged1'),
    },
    'diverged2': {
        'lik_mean_d': Envelope(0.0259, 0.6311, 0.4918, 247, 'diverged2'),
        'lik_frac_near': Envelope(0.0702, 1.0000, 0.2333, 247, 'diverged2'),
        'cov_sigma_xy': Envelope(0.0679, 1.3687, 0.4758, 247, 'diverged2'),
        'err_xy_ground_truth': Envelope(0.3061, 3.9285, 3.2476, 247,
                                        'diverged2'),
    },
}

# The legs that finished, and the legs that did not. Named so a reader
# cannot mistake `healthy2` for a success on the strength of its name.
C2M50_SUCCEEDED = ('healthy1', 'obstacle1')
C2M50_FAILED = ('healthy2', 'diverged1', 'diverged2')

# Measured detection latency, seconds after the ground-truth error first
# exceeded 1 m on run `diverged1`. Recorded here because it is the whole
# argument for which signal is primary.
C2M50_LATENCY_S = {
    # run -> signal -> seconds after the ground-truth error first exceeded
    # 1 m before that signal left `healthy1`'s envelope. Two runs, so the
    # 0.4 s is replicated and the covariance figure is not a one-off.
    'diverged1': {'lik_mean_d': 0.4, 'lik_frac_near': 0.4,
                  'cov_sigma_xy': 24.5},
    'diverged2': {'lik_mean_d': 0.4, 'lik_frac_near': 3.7,
                  'cov_sigma_xy': 13.9},
}
