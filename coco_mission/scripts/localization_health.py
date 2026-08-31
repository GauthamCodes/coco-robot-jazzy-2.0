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

Same split as ``mission_states.py`` (pure) and ``mission_executive.py``
(the adapter), for the same reason: a rule you cannot test without Gazebo
is a rule nobody tests. This half stays pure — no ROS, no clock, no I/O —
and ``localization_monitor.py`` is the node that feeds it.

**C2-M5.0 shipped this file wired to nothing, deliberately, and C2-M5.1
wired it up.** The docstring below is C2-M5.0's characterization and
stands unchanged; the C2-M5.1 block at the foot of the file is where the
thresholds it refused to invent were finally named, and it shows the
replay that justifies each one. What C2-M5.0 said about *which signal*
was never revised: covariance is still not the divergence test.

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

from coco_config.robot import (
    PLATFORM_LEN,
    RAMP_FOOT_X,
    RAMP_RUN,
    RAMP_SUMMIT_X,
    RAMP_WIDTH,
)

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
    # None means "do not test the /amcl_pose gap at all". C2-M5.1
    # measured that a fixed bound here is wrong in principle, not merely
    # mistuned — see MAX_AMCL_AGE_NOT_A_TEST at the foot of this file.
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
    if (thresholds.max_amcl_age is not None
            and obs.amcl_age is not None
            and obs.amcl_age > thresholds.max_amcl_age):
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


# ═══════════════════════════════════════════════════════════════════════
# C2-M5.1 — the numbers, and where each one comes from
# ═══════════════════════════════════════════════════════════════════════
#
# C2-M5.0 refused to pick a threshold and said what would settle it. This
# is that decision, made on the evidence C2-M5.0 recorded and on nothing
# else. It was **not** found by a search: one candidate was proposed from
# the healthy maximum, replayed once over all five recorded runs, and
# kept. The replay is reproducible from the committed CSVs — see
# RESULTS.md, "C2-M5.1 the threshold, and how it was chosen".
#
# Replay of the RETURN_HOME leg of all five C2-M5.0 runs, gated to mapped
# ground, at `lik_mean_d > 0.40 m`:
#
#   run         finished   leg_s   gated max   excursions   longest
#   healthy1    yes         80.3      0.3139            0         --
#   obstacle1   yes         50.0      0.3851            0         --
#   healthy2    NO          12.0      0.3091            0         --
#   diverged1   NO         131.5      0.5569           15    11.47 s
#   diverged2   NO          24.7      0.5084            1     5.02 s
#
# So 0.40 is justified as "strictly above every gated sample recorded on
# a leg that finished" — the largest such sample is obstacle1's 0.3851 —
# and not as the midpoint of a gap, which is the number C2-M5.0 correctly
# refused to invent.
LIK_MEAN_D_MAX = 0.40

# The replay above, as data. **These are gated to mapped ground and
# C2M50_ENVELOPE is not**, which is exactly the distinction that matters:
# C2-M5.0 recorded whole-leg ranges including the ramp and the platform,
# where the metric is uninterpretable. Gating changes the numbers enough
# to change conclusions — diverged2's `lik_frac_near` floor is 0.0702
# over the whole leg and 0.2500 on mapped ground — so the two records are
# kept separate rather than one being quietly corrected into the other.
#
# Measured 2026-08-31 by replaying the committed c2m5_*.csv files. Not a
# new run: the same five legs C2-M5.0 recorded, read with the gate on.
C2M51_GATED = {
    #  run:         (samples, lik_mean_d hi, lik_frac_near lo, leg seconds)
    'healthy1': Envelope(0.0008, 0.3139, 0.3167, 463, 'healthy1 gated'),
    'obstacle1': Envelope(0.0052, 0.3851, 0.1795, 270, 'obstacle1 gated'),
    'healthy2': Envelope(0.0483, 0.3091, 0.2963, 62, 'healthy2 gated'),
    'diverged1': Envelope(0.0267, 0.5569, 0.0536, 785, 'diverged1 gated'),
    'diverged2': Envelope(0.0259, 0.5084, 0.2500, 110, 'diverged2 gated'),
}
# `Envelope.median` is carrying the gated `lik_frac_near` FLOOR here
# rather than a median, because that is the number the frac_near verdict
# rests on and inventing a second dataclass for one field would be worse.
# Named so nobody reads it as a median.
C2M51_GATED_FRAC_NEAR_LO = {
    run: env.median for run, env in C2M51_GATED.items()}

# How long the excursions above LIK_MEAN_D_MAX actually lasted, gated.
# Zero on both legs that finished; this is the false-positive evidence.
C2M51_EXCURSIONS = {
    'healthy1': (0, 0.0),
    'obstacle1': (0, 0.0),
    'healthy2': (0, 0.0),
    'diverged1': (15, 11.47),
    'diverged2': (1, 5.02),
}

# `lik_frac_near` is carried and NOT used, and that is a measurement.
# diverged2's minimum on mapped ground is 0.2500, which is HIGHER than
# obstacle1's 0.1795 — the signal orders the cleanest injected divergence
# *above* a leg that finished, so no threshold below the healthy floor can
# fire on it. Replayed at 0.15 it changes no verdict on any of the five
# runs, so it would buy nothing and could only false-positive later.
# Zero disables the comparison rather than leaving a live-looking knob
# that has been measured to be inert.
LIK_FRAC_NEAR_DISABLED = 0.0

# ── the /amcl_pose gap is NOT a staleness test, and this is measured ─────
#
# C2-M5.0 shipped `max_amcl_age = 5.0` as an obvious-looking default.
# **Experiment 1 measured it to be wrong in principle, not mistuned**, and
# it was the only false positive the whole mission produced.
#
# One healthy mission that completed, 171.2 s, monitor publishing and the
# executive told not to act on it:
#
#   INCONSISTENT samples on mapped ground   0
#   latched-degraded samples              405  (40.5 s)
#   distinct recovery triggers              3
#
# **Every one of the 405 was POSE_STALE and not one was SCAN_DISAGREES.**
# They fall in GRASP (255), PLACE (95), IDLE (29) — the states where the
# robot is standing still. nav2_params.yaml sets `amcl.update_min_d: 0.25`
# and `update_min_a: 0.2`, so AMCL runs a filter update, and therefore
# publishes /amcl_pose, only after the robot has MOVED. A stationary robot
# publishes nothing and the gap grows without bound. GRASP stands still
# for ~50 s. Treating an inter-message gap on an event-driven topic as
# staleness reports "stationary" as "lost".
#
# Raising the bound would be tuning a check whose premise is false: there
# is no value that distinguishes a long stationary grasp from a dead
# filter, because on this stack they produce the identical topic silence.
#
# What the same 405 rows show is that the other freshness signal was
# healthy throughout: `map_odom_age` held between -0.400 and -0.390 s, the
# -0.44 C2-M5.0 measured. map->odom is republished on AMCL's own schedule
# whether or not the robot moves, AMCL is its only publisher, and so an
# AMCL that has actually stopped drives that age up through zero. It
# covers the real failure, and its bound is the stack's own
# `transform_tolerance` rather than a number anybody invented.
#
# So `amcl_age` joins `cov_sigma_xy`: recorded because it is free and
# informative to a human, and **not consulted**.
MAX_AMCL_AGE_NOT_A_TEST = None

C2M51_THRESHOLDS = Thresholds(
    lik_mean_d_max=LIK_MEAN_D_MAX,
    lik_frac_near_min=LIK_FRAC_NEAR_DISABLED,
    max_amcl_age=MAX_AMCL_AGE_NOT_A_TEST,
    # min_beams and max_map_odom_age keep the values C2-M5.0 shipped. The
    # second is the only threshold in this file that was ever settled
    # without inventing anything: it is the stack's own
    # amcl.transform_tolerance, not a new constant.
)

# Experiment 1, recorded so the false-positive claim carries its run.
C2M51_EXP1 = {
    'result': 'COMPLETE',
    'sim_seconds': 171.2,
    'samples': 1714,
    'gated_samples': 882,
    'gated_lik_mean_d_max': 0.3430,
    'gated_lik_mean_d_p99': 0.3000,
    'inconsistent_on_mapped_ground': 0,
    # Before the amcl_age check was removed. Kept because deleting the
    # evidence that motivated a change is how a change stops being
    # justifiable later.
    'pose_stale_triggers_before_fix': 3,
    'scan_disagrees_triggers': 0,
}

# How long INCONSISTENT must hold before it means anything.
#
# Requirement 4 of C2-M5.0's recovery list: "the trigger needs
# persistence, not one sample". At LIK_MEAN_D_MAX neither leg that
# finished produces a single excursion, so the healthy data does not bound
# this from below and a search over it would be a search over noise. The
# bound that IS measured is from above: the shortest excursion on either
# injected divergence is diverged2's 5.02 s. 2.0 s sits at 40% of that —
# comfortably inside the shortest true positive, and 20 consecutive
# samples at the 10 Hz the recorder and the monitor both run at.
DEGRADED_HOLD_S = 2.0

# How long CONSISTENT must hold before the mission may resume.
#
# **Deliberately longer than DEGRADED_HOLD_S, and this is a design
# choice rather than a measurement.** The two errors are not symmetric:
# triggering recovery on a healthy robot costs a spin, and resuming a
# 3 m-wrong robot costs the mission. C2-M5.0 measured no resume criterion
# at all — it implemented no recovery — so there is no run to read this
# off, and it is recorded here as an assumption rather than dressed up as
# evidence.
HEALTHY_HOLD_S = 3.0


# ── the mapped-ground gate, in world coordinates ─────────────────────────
# `coco_world.pgm` is a 2D slice of the FLAT world: the ramp, the platform
# and the far slope are not in it, so a scan taken on any of them
# disagrees with the map for a reason that is not localization. The span
# is derived from coco_config rather than typed, so a re-parameterised
# wedge moves the gate with it.
#
# foot 1.0 --- crest 3.0 === platform 4.5 --- far foot 6.5
MAPPED_GROUND_MIN_X = RAMP_FOOT_X
MAPPED_GROUND_MAX_X = RAMP_SUMMIT_X + PLATFORM_LEN + RAMP_RUN
# The wedge is only RAMP_WIDTH across, centred on y=0. **The gate was
# x-only until Experiment 2 measured what that costs**, and it cost the
# whole return leg: the robot does not climb back over the wedge to get
# home, it drives AROUND it, down a corridor at |y| ~ 2 that is ordinary
# mapped floor. An x-only gate calls that corridor unmapped and throws
# the signal away exactly where C2-M5 needs it. Measured on exp2d: 65% of
# RETURN_HOME gated out, 48 INCONSISTENT samples ignored, no trigger.
#
# Whether the corridor is scoreable at all was a real question -- the
# laser sees the wedge's flank from there, and the wedge is not in the
# map -- so it was measured on the five C2-M5.0 runs rather than assumed.
# Worst corridor sample on a leg that FINISHED: 0.3798 (obstacle1),
# against 0.3851 on the flat. The corridor behaves like the flat, and it
# is where diverged2 kept its strongest evidence: 137 samples, median
# 0.5075, all of which the x-only gate discarded.
MAPPED_GROUND_HALF_WIDTH = RAMP_WIDTH / 2.0


def on_mapped_ground(world_x, world_y=None):
    """True where the 2D map describes the ground the scan is hitting.

    Takes the robot's OWN estimate of where it is. That is the honest
    input: a robot that is lost may gate wrongly, and gating on ground
    truth would answer a question the robot cannot ask. The failure mode
    is benign in the direction that matters — a lost robot that believes
    it is on the ramp suppresses its own alarm, which is why the gate is
    a suppressor of noise and never a source of confidence.

    ``world_y`` omitted falls back to the x-only test, which is the
    conservative reading: it gates out the corridor as well as the wedge.
    Callers that know where they are laterally should say so.
    """
    if world_x is None:
        return False
    if not (MAPPED_GROUND_MIN_X <= world_x <= MAPPED_GROUND_MAX_X):
        return True
    # Inside the wedge's x-span. Only the wedge itself is missing from
    # the map; the floor either side of it is in the map like any other.
    if world_y is None:
        return False
    return abs(world_y) > MAPPED_GROUND_HALF_WIDTH


class Persistence:
    """Latch a condition once the evidence for it has accumulated.

    Pure: it is handed a time and a boolean and keeps no clock of its own,
    which is what lets the tests drive it through a whole excursion in a
    loop with no ROS and no sleeping.

    **This began as a strict-contiguity rule and Experiment 2 measured
    that to be wrong in kind, not mistuned.** The first version reset the
    run on any single false sample. On the live injected divergence the
    scan signal dithers across its threshold at 10 Hz — 81 INCONSISTENT
    samples inside RETURN_HOME, but the longest *unbroken* stretch was
    **1.80 s** against a 2.0 s hold, so a real 3 m divergence never
    latched. The same stretch held ≥80% INCONSISTENT for **4.60 s**. The
    evidence was there; the debouncer was throwing it away on one good
    sample in five.

    Lowering the hold to fit 1.80 s would have been tuning a constant to
    make one run pass. This instead changes the *rule*: evidence
    accumulates while the condition holds and drains while it does not,
    both at wall rate, capped at ``hold``. So

    * sustained true  → latches after exactly ``hold`` seconds, which is
      what ``hold`` meant before and still means
    * sustained false → clears after ``hold`` seconds
    * 80% true        → accumulates at 0.6 s per second, latching in
      ``hold``/0.6; on the Experiment 2 stretch, 3.3 s inside a 4.6 s
      window
    * 50/50 noise     → never latches, at any duration

    The hysteresis is the point: a condition that is merely *usually*
    true still has to earn the latch, and one good sample cannot spend
    the evidence that ten bad ones bought.
    """

    __slots__ = ('hold', '_credit', '_last', '_latched')

    # A gap longer than this is treated as a gap, not as evidence. Without
    # it, a monitor that was starved for ten seconds resumes and applies
    # ten seconds of credit in one update.
    MAX_STEP = 1.0

    def __init__(self, hold):
        self.hold = hold
        self._credit = 0.0
        self._last = None
        self._latched = False

    def update(self, now, holds):
        step = 0.0 if self._last is None else min(now - self._last,
                                                  self.MAX_STEP)
        self._last = now
        if step < 0.0:                       # clock went backwards
            step = 0.0
        self._credit += step if holds else -step
        self._credit = max(0.0, min(self.hold, self._credit))
        if self._credit >= self.hold:
            self._latched = True
        elif self._credit <= 0.0:
            self._latched = False
        return self._latched

    def reset(self):
        self._credit = 0.0
        self._last = None
        self._latched = False

    @property
    def latched(self):
        return self._latched

    @property
    def credit(self):
        """Seconds of net evidence accumulated, 0..hold."""
        return self._credit

    def held_for(self, now):
        """Seconds of net evidence, for the status line."""
        return self._credit


class LikelihoodField:
    """Distance, in metres, from a world point to the nearest occupied cell.

    The same field ``nav2_amcl`` scores particles against
    (``laser_model_type: likelihood_field``) and never publishes.
    ``c2m5_locrec.py`` computes it from a map YAML for offline recording;
    this takes the occupancy grid the robot was handed at runtime, so the
    node needs no file path and no second copy of the map.

    Points outside the map come back NaN rather than clamped: a beam that
    leaves the map is not evidence of a good pose or a bad one, and
    averaging a made-up number in would be the same mistake as inventing
    a threshold.
    """

    def __init__(self, occupied, resolution, origin):
        # Imported here, not at module scope, so the pure module stays
        # importable — and stays TESTABLE — on a machine with neither.
        import numpy as np
        from scipy.ndimage import distance_transform_edt

        self.occupied = np.asarray(occupied, dtype=bool)
        self.h, self.w = self.occupied.shape
        self.res = float(resolution)
        self.origin = (float(origin[0]), float(origin[1]))
        self.dist = distance_transform_edt(~self.occupied) * self.res
        self.n_occupied = int(self.occupied.sum())

    @classmethod
    def from_occupancy_grid(cls, data, width, height, resolution, origin,
                            occupied_thresh=65):
        """Build from a ``nav_msgs/OccupancyGrid``'s own fields.

        ``data`` is the message's row-major 0-100 occupancy with -1 for
        unknown. Unknown is NOT occupied: an unmapped cell is an absence
        of evidence, and treating it as an obstacle would make every
        endpoint in unexplored space look explained.
        """
        import numpy as np

        grid = np.asarray(data, dtype=np.int16).reshape(height, width)
        return cls(grid >= occupied_thresh, resolution, origin)

    def distance(self, xs, ys):
        """Metres to the nearest occupied cell, NaN outside the map."""
        import numpy as np

        j = np.floor((np.asarray(xs) - self.origin[0]) / self.res)
        i = np.floor((np.asarray(ys) - self.origin[1]) / self.res)
        inside = (i >= 0) & (i < self.h) & (j >= 0) & (j < self.w)
        out = np.full(np.shape(xs), np.nan, dtype=np.float64)
        if inside.any():
            out[inside] = self.dist[i[inside].astype(int),
                                    j[inside].astype(int)]
        return out


# Beams per scan scored against the field. nav2_amcl's own `max_beams` is
# 60; matching it keeps the number comparable to what the filter itself
# scores. Same value as c2m5_locrec.py, so the monitor's live figure and
# the recorded CSVs mean the same thing.
LIK_BEAMS = 60
# An endpoint this close to an occupied cell counts as "explained".
# 0.10 m is two map cells at the map's 0.05 m resolution.
LIK_NEAR_M = 0.10


def score_scan(field, ranges, angle_min, angle_increment,
               range_min, range_max, tx, ty, theta, beams=LIK_BEAMS):
    """Place scan endpoints with a map->laser pose and score them.

    Returns ``(mean_d, frac_near, n)``, or ``(None, None, 0)`` when
    nothing scoreable survived. Pure arithmetic over the field: the
    caller supplies the transform, so this never touches TF.
    """
    import numpy as np

    n = len(ranges)
    if field is None or n == 0:
        return None, None, 0
    step = max(1, n // beams)
    idx = np.arange(0, n, step)
    r = np.asarray(ranges, dtype=np.float64)[idx]
    a = angle_min + idx * angle_increment
    good = np.isfinite(r) & (r > range_min) & (r < range_max)
    if not good.any():
        return None, None, 0
    r, a = r[good], a[good]
    d = field.distance(tx + r * np.cos(theta + a),
                       ty + r * np.sin(theta + a))
    d = d[np.isfinite(d)]
    if d.size == 0:
        return None, None, 0
    return float(d.mean()), float((d <= LIK_NEAR_M).mean()), int(d.size)
