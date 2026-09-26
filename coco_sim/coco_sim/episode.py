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
The episode specification — one task instance, reproducible from a seed.

An *episode* is one complete COCO task: a world, a set of targets, a
requested target, a robot start pose, and the metadata needed to replay
it. This module resolves one from a seed and nothing else.

Why it lives in ``coco_sim``
----------------------------
``coco_sim`` already owns the repository's only seeded generator
(:func:`coco_sim.yard.sample_yard`) and depends on ``coco_config`` alone,
so putting the episode layer here keeps the package graph acyclic
(CLAUDE.md rule 6). ``coco_config`` stays what it is — constants — and
the mission, the web platform and the RL envs can all read a manifest
without any of them depending on each other.

The two halves of an episode, and why they are separated
--------------------------------------------------------
**The manifest is privileged. The task view is not.**

:meth:`EpisodeSpec.manifest` carries everything: where each cylinder
actually is, which obstacles exist, what the seed was. That is for world
generation, evaluation, dataset annotation, regression testing and
debugging.

:meth:`EpisodeSpec.task_view` carries the episode id and the requested
colour. That is *all* the robot is entitled to. It contains no
coordinate of any kind, and a test asserts that — because the moment a
pose leaks into the robot's input, every perception and navigation
result measured afterwards is measuring the wrong thing. The robot is
told *what to find*, never *where it is*.

This is the distinction that the frozen P0.2 mission does not yet make:
there, ``lane_for_colour()`` hands the sequencer a y-coordinate at
compile time. This module does not remove that — P0.2 stays exactly as
it is — it builds the representation that lets it be removed later.

Determinism
-----------
One :class:`random.Random`, seeded once, drives every draw. Same seed,
same episode, on any machine and any backend. Stdlib rather than
``numpy.random`` deliberately: ``yard.py`` needs numpy because it
samples a heightfield array, and a manifest layer that every consumer —
the browser platform included — must import numpy to read is a worse
trade than one that needs nothing.

What this module does NOT do
----------------------------
It does not spawn anything, write a world file, or talk to a simulator.
It resolves a specification and validates it. Translating a manifest
into one simulator's objects is :mod:`coco_sim.backends`' job — Gazebo
SDF and spawn poses, an Isaac prim list — and the point of keeping this
module backend-agnostic is that both translate the SAME manifest.

Regions (stage C)
-----------------
Every target stands in a named region (``coco_config.robot
.TARGET_REGIONS``: the four lanes, with the colour taken off them). The
manifest records colour -> region, and that assignment — not
``lane_for_colour`` — is what an episode varies. The compatibility
mission is handed region NAMES (:func:`compat_mission_inputs`), never a
pose; ``task_view()`` is unchanged.
"""

from dataclasses import asdict, dataclass, field, replace
import hashlib
import json
import math
import random

from coco_config.robot import (approach_stop_x, approach_window,
                               FIXED_REGION_MAP, format_region_map,
                               PLATFORM_LEN, RAMP_ANGLE_DEG, RAMP_RUN,
                               RAMP_SUMMIT_X, RAMP_WIDTH, region_by_id,
                               SPAWN_XY, SPAWN_Z, target_by_colour,
                               TARGET_COLOURS, TARGET_REGIONS, TARGETS,
                               WHEEL_RADIUS, WHEEL_SEPARATION, WHEEL_WIDTH,
                               WHEELBASE)

# ── the physical envelope, DERIVED, never re-typed ───────────────────────
# CLAUDE.md rule 3 and the randomiser brief both say the same thing: the
# generator must not carry its own copy of the robot's geometry. Every
# bound below is computed from coco_config at import time, so a change
# there moves the envelope here and the tests that pin it.

#: Half the robot's x-footprint: wheel joints at +-WHEELBASE/2, each wheel
#: WHEEL_RADIUS beyond that. Equals 0.1485 m, which is the literal
#: ``HALF_FOOTPRINT_X`` in ``coco_config/test/test_targets.py`` — derived
#: here rather than copied, so the two cannot drift apart.
HALF_FOOTPRINT_X = WHEELBASE / 2.0 + WHEEL_RADIUS

#: Half the robot's y-footprint: wheel separation plus the wheels' width.
HALF_FOOTPRINT_Y = WHEEL_SEPARATION / 2.0 + WHEEL_WIDTH / 2.0


def platform_top_z(ramp_angle_deg=RAMP_ANGLE_DEG):
    """World z of the platform's top surface at `ramp_angle_deg`.

    The wedge rises over RAMP_RUN and the platform sits flush with its
    crest. The same expression full_world_robo.launch.py builds the
    platform from, so a target's manifest z and the platform it stands
    on agree to the bit at any grade the launch file accepts.
    """
    return RAMP_RUN * math.tan(math.radians(ramp_angle_deg))


#: World z of the platform's top surface at the default grade.
PLATFORM_TOP_Z = platform_top_z()

#: World x of the platform's near and far edges.
PLATFORM_NEAR_X = RAMP_SUMMIT_X
PLATFORM_FAR_X = RAMP_SUMMIT_X + PLATFORM_LEN

#: Backends an episode may name. The manifest is engine-agnostic; this
#: field records which engine a given run used so a result can be
#: attributed. Physics is NOT claimed to be identical across them — that
#: difference is the thing later stages are meant to measure, not hide.
BACKENDS = ('gazebo', 'isaac', 'mujoco')

#: World variants this module knows how to validate against. ``coco_world``
#: is the frozen v1 fetch arena; ``coco_yard`` is the M7 terrain world,
#: whose geometry is owned by :mod:`coco_sim.yard` and is not re-declared
#: here.
WORLD_VARIANTS = ('coco_world', 'coco_yard')

#: How much a target may be randomised in the ``'positions'`` level, as a
#: fraction of the distance to its validated bound. Kept well under 1.0
#: so a generated layout is never one floating-point step from illegal.
PLACEMENT_FILL = 0.85

#: Deterministic rejection sampling gives up after this many draws rather
#: than looping forever on an over-constrained envelope.
MAX_PLACEMENT_ATTEMPTS = 200

#: Randomisation levels, weakest first. ``fixed`` reproduces the frozen
#: P0.2 layout exactly and is the default, so an episode-aware caller
#: that asks for nothing gets today's behaviour.
LEVELS = ('fixed', 'colours', 'positions')

# ── the region placement area (stage C) ──────────────────────────────────
# Every target stands in a named region (coco_config.robot.TARGET_REGIONS)
# and the compatibility mission reaches it by climbing that region's lane
# — it is told the region, never the pose. So a target may only move as
# far from its region's nominal pose as the robot's approach, starting
# from that lane, has been SHOWN to absorb. Two bounds, both one-sided in
# the direction the evidence supports:
#
# Across the lane. approach_server's align phase nulls the bearing before
# the fix the grasp uses is taken, so lateral offset is absorbed rather
# than carried: +0.030 m commanded arrived at the grasp as -3.0 mm and
# grasped, verified (C2-M4.1, PROJECT_STATE.md, n = 1). 30 mm is the
# largest offset tried, NOT a characterised limit, which is exactly why it
# is the bound: it is where the evidence stops.
#
# Along the lane. Only AWAY from the crest, from the frozen row out to
# PLACEMENT_FILL of the platform's far bound. A nearer target would
# shorten the blind crest drive's clearance and the camera's stand-off at
# the start of the servo below what every measured fetch had; a farther
# one only lengthens the closed-loop servo, and the farthest legal target
# stays inside target_finder's 2.0 m range gate from the end of the climb.
# That is geometry (derived), not a measurement: no target off the frozen
# row had been driven to when this bound was set.
#
# Inside this area the p03 envelope (reachable row, standable band,
# separation, approach corridor) is still checked, and still binds first.

#: Half-width of a region's placement area across its lane, metres.
REGION_LATERAL_LIMIT = 0.030


class InvalidEpisode(ValueError):
    """An episode that violates a physical or schema constraint.

    Raised by :func:`validate_episode` and by the generator when
    rejection sampling cannot find a legal layout. It is deliberately a
    ``ValueError``: a caller that does not catch it gets a crash rather
    than a silently unreachable target.
    """


@dataclass(frozen=True)
class TargetSpec:
    """One fetch target, resolved to a world pose.

    ``colour`` is the only field the robot may ever see, and it sees it
    through :meth:`EpisodeSpec.task_view`, not through this object.
    Everything else is privileged: it exists so the world can be built
    and the outcome scored.
    """

    colour: str
    model: str
    x: float
    y: float
    z: float
    diameter: float
    height: float
    #: The named region (coco_config.robot.TARGET_REGIONS) this target
    #: stands in. The manifest is the source of truth for colour->region;
    #: the pose must lie inside the region's area, which
    #: :func:`validate_episode` checks. Last and defaulted so a manifest
    #: written before regions existed still loads — and then fails
    #: validation, loudly, rather than being guessed at.
    region_id: str = ''

    @property
    def radius(self):
        """Half the cylinder's diameter, in metres."""
        return self.diameter / 2.0


@dataclass(frozen=True)
class ObstacleSpec:
    """A static or dynamic obstacle the episode adds to the world.

    Dynamic motion is *represented* here and generated nowhere: every
    obstacle this module produces today has ``motion='static'`` and the
    generator adds none at all. The fields exist so that when dynamic
    obstacles arrive they extend a schema that is already serialised,
    tested and reproducible, instead of forcing a manifest migration.

    ``enabled`` is honoured by whatever consumes the manifest; a
    disabled obstacle stays in the record so a run remains reproducible
    after someone turns it off.
    """

    name: str
    kind: str                      # 'box' | 'cylinder'
    x: float
    y: float
    z: float
    yaw: float = 0.0
    size: tuple = ()               # box: (sx, sy, sz); cylinder: (r, len)
    motion: str = 'static'
    enabled: bool = True
    waypoints: tuple = ()
    speed: float = 0.0


@dataclass(frozen=True)
class RobotStart:
    """Where the robot begins the episode, in world coordinates."""

    x: float
    y: float
    z: float
    yaw: float = 0.0


@dataclass(frozen=True)
class EpisodeSpec:
    """One complete, reproducible COCO task instance.

    Build one with :func:`generate_episode`; do not construct it field by
    field unless you are writing a test, because the constructor does not
    validate. :func:`validate_episode` does, and the generator always
    calls it before returning.
    """

    episode_id: str
    seed: int
    level: str
    backend: str
    world_variant: str
    ramp_angle_deg: float
    requested_colour: str
    targets: tuple
    obstacles: tuple
    robot_start: RobotStart
    metadata: dict = field(default_factory=dict)

    # ── the privileged/observable boundary ───────────────────────────────
    def manifest(self):
        """Return the full privileged record: everything, including poses.

        For world generation, evaluation, dataset annotation and
        debugging. Never hand this to the robot.
        """
        out = asdict(self)
        out['targets'] = [asdict(t) for t in self.targets]
        out['obstacles'] = [asdict(o) for o in self.obstacles]
        out['robot_start'] = asdict(self.robot_start)
        return out

    def task_view(self):
        """Return what the robot may know: the ask, and nothing else.

        Deliberately two fields. The robot is told which colour to fetch
        and which episode it is in; it must discover everything else
        through its own sensors. No coordinate, no lane, no target count
        and no world geometry appears here, and
        ``test_episode.py::test_the_task_view_leaks_no_geometry`` asserts
        that against every field of every target.

        ``backend`` is withheld on purpose too: a robot that can branch
        on which simulator it is in is a robot whose results do not
        transfer.
        """
        return {
            'episode_id': self.episode_id,
            'requested_colour': self.requested_colour,
        }

    def target(self, colour):
        """Look up the :class:`TargetSpec` of this colour, or None.

        Privileged — this is a manifest lookup, not a perception result.
        """
        for spec in self.targets:
            if spec.colour == colour:
                return spec
        return None

    def region_map(self):
        """Return which region each colour stands in: ``{colour: region_id}``.

        PRIVILEGED, and deliberately not part of :meth:`task_view`. It
        answers "where is the red one" at the granularity of a lane, which
        is exactly what the robot is supposed to find out for itself.

        It exists for one consumer: the compatibility mission, which must
        pick a lane on the flat before it can see anything on the platform
        (the crest occludes it; see ``coco_config.robot.lane_for_colour``).
        That mission is handed region NAMES through
        :func:`compat_mission_inputs` and resolves them against the static
        region table — it never receives a target coordinate. When the
        mission searches instead of looking up (stage F), this channel is
        what gets deleted.
        """
        return {t.colour: t.region_id for t in self.targets}

    # ── serialisation ────────────────────────────────────────────────────
    def to_json(self, indent=2):
        """Serialise the manifest to a JSON string with sorted keys.

        Sorted so that two runs of the same seed produce byte-identical
        text, which is what makes ``diff`` a usable regression check.
        """
        return json.dumps(self.manifest(), indent=indent, sort_keys=True)


def episode_id(seed, level, backend, world_variant, requested_colour):
    """Derive a short, stable id from the fields that define an episode.

    Derived rather than random so that the id is itself reproducible: two
    machines generating the same episode agree on what to call it, and an
    id in a results table is enough to regenerate the run.
    """
    key = f'{seed}|{level}|{backend}|{world_variant}|{requested_colour}'
    digest = hashlib.sha256(key.encode('utf-8')).hexdigest()
    return f'ep-{digest[:12]}'


# ── validation ───────────────────────────────────────────────────────────
def target_x_bounds(diameter):
    """Legal world-x range for a target of this diameter, as (near, far).

    Two constraints, both derived:

    - **Near.** The robot has to stand in front of the target with all
      four wheels on the platform. It stops ``approach_stop_x`` from the
      cylinder's axis and is ``HALF_FOOTPRINT_X`` long behind that, so
      the target cannot be closer to the crest than the sum.
    - **Far.** The cylinder's body must sit on the platform, so its far
      face cannot overhang the far edge.

    ``approach_stop_x`` is colour-independent in practice (the 5.5 mm
    window is identical for all four, see ``coco_config.robot``), but it
    is read per call rather than assumed so this stays correct if the
    self-collision bound is ever re-measured.
    """
    stop = max(approach_stop_x(t.colour) for t in TARGETS)
    near = PLATFORM_NEAR_X + stop + HALF_FOOTPRINT_X
    far = PLATFORM_FAR_X - diameter / 2.0
    return near, far


def target_y_bounds():
    """Legal world-y range for a target, as (low, high).

    The robot must be able to stand in the target's lane with its
    footprint on the platform, so the bound is the platform half-width
    less the robot's half-footprint — the same rule
    ``coco_config/test/test_targets.py::test_every_lane_fits_on_the_platform``
    already applies to the frozen lanes.
    """
    limit = RAMP_WIDTH / 2.0 - HALF_FOOTPRINT_X
    return -limit, limit


def min_target_separation(a, b):
    """Smallest legal centre distance between two targets, in metres.

    The robot has to be able to occupy one target's lane without its
    body touching the neighbour, so the bound is the robot's
    half-footprint across y plus both radii. Derived, so a wider chassis
    or a fatter cylinder tightens it automatically.
    """
    return HALF_FOOTPRINT_Y + a.radius + b.radius


def approach_corridor_blocked(far, near):
    """Say whether `near` stands in the robot's straight approach to `far`.

    The robot reaches every target the same way: over the crest, then
    straight along +x in the target's own y. Its body sweeps a corridor
    ``HALF_FOOTPRINT_Y`` either side of that line, so any cylinder closer
    to the crest (smaller x) whose body intrudes on the corridor is hit
    before the target is reached — and, from the crest camera, stands in
    front of it.

    Pairwise distance does NOT catch this: two cylinders 0.9 m apart in
    x at the same y pass a separation check and still block each other.
    Measured on the generator before this rule existed: 1012 of 10000
    ``positions`` episodes had a blocked corridor, 276 of them the
    requested target's. The frozen layout and the ``colours`` level had
    none, because every target sits at the same x.
    """
    return (near.x < far.x
            and abs(near.y - far.y) < HALF_FOOTPRINT_Y + near.radius)


def region_area(region, diameter):
    """Where a target of `diameter` may stand in `region`.

    Returns ``((x_low, x_high), (y_low, y_high))`` in world metres. The
    region's nominal pose ``(row_x, lane_y)`` is inside it — on the near
    x edge, centred in y — so ``fixed`` and ``colours`` layouts, which
    put every target exactly on its nominal pose, are legal by the same
    rule the ``positions`` level draws against. See the block comment on
    :data:`REGION_LATERAL_LIMIT` for why each bound is where it is.
    """
    _, far = target_x_bounds(diameter)
    x_high = region.row_x + (far - region.row_x) * PLACEMENT_FILL
    return ((region.row_x, x_high),
            (region.lane_y - REGION_LATERAL_LIMIT,
             region.lane_y + REGION_LATERAL_LIMIT))


#: Float slack on the region-area comparison, so a pose drawn exactly on
#: a bound and round-tripped through JSON is not rejected by the last bit.
_AREA_EPS = 1e-9


def validate_episode(spec):
    """Raise :class:`InvalidEpisode` if `spec` is not physically legal.

    Checks the schema first (cheap, and a bad enum makes the geometry
    checks meaningless), then every placement against the derived
    envelope. Returns None; it is used for its exception.
    """
    # ── schema ───────────────────────────────────────────────────────────
    if spec.level not in LEVELS:
        raise InvalidEpisode(
            f'unknown level {spec.level!r}, expected one of {LEVELS}')
    if spec.backend not in BACKENDS:
        raise InvalidEpisode(
            f'unknown backend {spec.backend!r}, expected one of {BACKENDS}')
    if spec.world_variant not in WORLD_VARIANTS:
        raise InvalidEpisode(
            f'unknown world variant {spec.world_variant!r}, '
            f'expected one of {WORLD_VARIANTS}')
    if not isinstance(spec.seed, int):
        raise InvalidEpisode(f'seed must be an int, got {type(spec.seed)}')
    if not spec.targets:
        raise InvalidEpisode('an episode with no targets has no task')

    colours = [t.colour for t in spec.targets]
    if len(set(colours)) != len(colours):
        raise InvalidEpisode(f'duplicate target colours: {colours}')
    for colour in colours:
        if colour not in TARGET_COLOURS:
            raise InvalidEpisode(
                f'{colour!r} is not a known target colour {TARGET_COLOURS}')
    if spec.requested_colour not in colours:
        raise InvalidEpisode(
            f'requested colour {spec.requested_colour!r} is not present '
            f'in this episode ({colours})')

    # ── placement ────────────────────────────────────────────────────────
    y_low, y_high = target_y_bounds()
    for target in spec.targets:
        window = approach_window(target.colour)
        if window is None or window[0] >= window[1]:
            raise InvalidEpisode(
                f'{target.colour} has no graspable approach window')

        near, far = target_x_bounds(target.diameter)
        if not near <= target.x <= far:
            raise InvalidEpisode(
                f'{target.colour} at x={target.x:.4f} is outside the '
                f'reachable row [{near:.4f}, {far:.4f}]')
        if not y_low <= target.y <= y_high:
            raise InvalidEpisode(
                f'{target.colour} at y={target.y:+.4f} is outside the '
                f'standable band [{y_low:+.4f}, {y_high:+.4f}]')

        expected_z = platform_top_z(spec.ramp_angle_deg) + target.height / 2.0
        if abs(target.z - expected_z) > 1e-6:
            raise InvalidEpisode(
                f'{target.colour} at z={target.z:.4f} does not rest on '
                f'the platform (expected {expected_z:.4f})')

    for i, a in enumerate(spec.targets):
        for b in spec.targets[i + 1:]:
            gap = math.hypot(a.x - b.x, a.y - b.y)
            floor = min_target_separation(a, b)
            if gap < floor:
                raise InvalidEpisode(
                    f'{a.colour} and {b.colour} are {gap:.4f} m apart, '
                    f'closer than the {floor:.4f} m the robot needs')

    for far in spec.targets:
        for near in spec.targets:
            if near is not far and approach_corridor_blocked(far, near):
                raise InvalidEpisode(
                    f'{near.colour} stands in the approach corridor of '
                    f'{far.colour} ({abs(near.y - far.y):.4f} m off its '
                    f'line, closer to the crest)')

    # ── regions (stage C) ────────────────────────────────────────────────
    # Checked LAST on purpose: every rule above is the physical envelope
    # and says something more specific about a bad pose. These say the
    # pose is legal but not where the region the mission will climb to
    # can deliver the robot.
    seen = {}
    for target in spec.targets:
        region = region_by_id(target.region_id)
        if region is None:
            raise InvalidEpisode(
                f'{target.colour} names no known region '
                f'({target.region_id!r}); every target must stand in one')
        if target.region_id in seen:
            raise InvalidEpisode(
                f'{target.colour} and {seen[target.region_id]} share '
                f'region {target.region_id}')
        seen[target.region_id] = target.colour
        (x_low, x_high), (y_low, y_high) = region_area(region,
                                                       target.diameter)
        if not (x_low - _AREA_EPS <= target.x <= x_high + _AREA_EPS
                and y_low - _AREA_EPS <= target.y <= y_high + _AREA_EPS):
            raise InvalidEpisode(
                f'{target.colour} at ({target.x:.4f}, {target.y:+.4f}) is '
                f'outside region {target.region_id} '
                f'x [{x_low:.4f}, {x_high:.4f}] '
                f'y [{y_low:+.4f}, {y_high:+.4f}]')


# ── generation ───────────────────────────────────────────────────────────
def _fixed_layout(top_z=PLATFORM_TOP_Z):
    """Return the frozen P0.2 layout, straight out of ``coco_config``."""
    out = []
    for t in TARGETS:
        region = region_by_id(FIXED_REGION_MAP[t.colour])
        out.append(TargetSpec(colour=t.colour, model=t.model,
                              x=region.row_x, y=region.lane_y,
                              z=top_z + t.height / 2.0,
                              diameter=t.diameter, height=t.height,
                              region_id=region.region_id))
    return out


def _permuted_colours(rng):
    """Shuffle the four colours — level 1.

    Which *region* a colour occupies changes; the set does not. That is
    enough on its own to break a compile-time colour->lane lookup, which
    is the whole point of the level.
    """
    colours = list(TARGET_COLOURS)
    rng.shuffle(colours)
    return colours


def _jittered(rng, diameter, region):
    """One (x, y) draw inside `region`'s placement area — level 2.

    The area is :func:`region_area`: along the lane only away from the
    crest, across it only as far as the approach has been measured to
    absorb. x is drawn before y, from the episode's one generator.
    """
    (x_low, x_high), (y_low, y_high) = region_area(region, diameter)
    x = rng.uniform(x_low, x_high)
    y = rng.uniform(y_low, y_high)
    return x, y


def generate_episode(seed=0, level='fixed', backend='gazebo',
                     world_variant='coco_world', requested_colour=None,
                     ramp_angle_deg=RAMP_ANGLE_DEG, obstacles=(),
                     metadata=None):
    """Resolve one episode. Deterministic given `seed` and `level`.

    ``level='fixed'`` (the default) reproduces the frozen P0.2 layout
    exactly, so this function can be introduced alongside the existing
    mission without changing a single spawned pose. The randomised
    levels are opt-in:

    - ``'colours'`` permutes which colour occupies which region, each
      target on its region's nominal pose;
    - ``'positions'`` permutes colours *and* moves each target inside
      its region's placement area (:func:`region_area`), by
      deterministic rejection sampling against the full envelope.

    The colour permutation is the first draw at both randomised levels,
    so a seed assigns the same colours to the same regions at either.

    `requested_colour` defaults to a seeded draw from the episode's own
    targets, so a bare ``generate_episode(seed=n)`` is already a complete
    task. Pass one explicitly to pin it.

    Raises :class:`InvalidEpisode` if no legal layout is found, rather
    than returning one that cannot be grasped.
    """
    if level not in LEVELS:
        raise InvalidEpisode(
            f'unknown level {level!r}, expected one of {LEVELS}')

    rng = random.Random(seed)
    top_z = platform_top_z(ramp_angle_deg)

    if level == 'fixed':
        targets = _fixed_layout(top_z)
    else:
        colours = _permuted_colours(rng)
        targets = []
        for colour, region in zip(colours, TARGET_REGIONS):
            base = target_by_colour(colour)
            if level == 'colours':
                x, y = region.row_x, region.lane_y
            else:
                x, y = _place(rng, base, region, targets, top_z)
            targets.append(TargetSpec(
                colour=colour, model=base.model, x=x, y=y,
                z=top_z + base.height / 2.0,
                diameter=base.diameter, height=base.height,
                region_id=region.region_id))

    if requested_colour is None:
        requested_colour = rng.choice([t.colour for t in targets])

    spec = EpisodeSpec(
        episode_id=episode_id(seed, level, backend, world_variant,
                              requested_colour),
        seed=seed,
        level=level,
        backend=backend,
        world_variant=world_variant,
        ramp_angle_deg=ramp_angle_deg,
        requested_colour=requested_colour,
        targets=tuple(targets),
        obstacles=tuple(obstacles),
        robot_start=RobotStart(x=SPAWN_XY[0], y=SPAWN_XY[1], z=SPAWN_Z),
        metadata=dict(metadata or {}),
    )
    validate_episode(spec)
    return spec


def _place(rng, base, region, placed, top_z=PLATFORM_TOP_Z):
    """Rejection-sample one legal (x, y) for `base`, or give up loudly.

    Deterministic: every draw comes from the episode's single generator,
    so the same seed rejects and accepts the same candidates in the same
    order.
    """
    for _ in range(MAX_PLACEMENT_ATTEMPTS):
        x, y = _jittered(rng, base.diameter, region)
        candidate = TargetSpec(
            colour=base.colour, model=base.model, x=x, y=y,
            z=top_z + base.height / 2.0,
            diameter=base.diameter, height=base.height,
            region_id=region.region_id)
        if all(math.hypot(candidate.x - other.x, candidate.y - other.y)
               >= min_target_separation(candidate, other)
               and not approach_corridor_blocked(candidate, other)
               and not approach_corridor_blocked(other, candidate)
               for other in placed):
            return x, y
    raise InvalidEpisode(
        f'no legal placement for {base.colour} after '
        f'{MAX_PLACEMENT_ATTEMPTS} attempts; the envelope is over-constrained')


def episode_from_manifest(data):
    """Rebuild an :class:`EpisodeSpec` from :meth:`EpisodeSpec.manifest`.

    Round-trips exactly, so a recorded episode can be replayed without
    the seed — which matters for an episode that was hand-edited to
    reproduce a specific failure.
    """
    return EpisodeSpec(
        episode_id=data['episode_id'],
        seed=data['seed'],
        level=data['level'],
        backend=data['backend'],
        world_variant=data['world_variant'],
        ramp_angle_deg=data['ramp_angle_deg'],
        requested_colour=data['requested_colour'],
        targets=tuple(TargetSpec(**t) for t in data['targets']),
        obstacles=tuple(ObstacleSpec(**_tuples(o))
                        for o in data['obstacles']),
        robot_start=RobotStart(**data['robot_start']),
        metadata=dict(data.get('metadata') or {}),
    )


def episode_from_json(text):
    """Rebuild an :class:`EpisodeSpec` from :meth:`EpisodeSpec.to_json`."""
    return episode_from_manifest(json.loads(text))


def _tuples(obstacle):
    """JSON turns tuples into lists; put them back."""
    out = dict(obstacle)
    out['size'] = tuple(out.get('size') or ())
    out['waypoints'] = tuple(tuple(w) for w in out.get('waypoints') or ())
    return out


def rebind(spec, **changes):
    """Return a copy of `spec` with `changes` applied, re-validated.

    The episode id is recomputed, because an episode whose backend or
    requested colour changed is a different episode and must not answer
    to the old name.
    """
    updated = replace(spec, **changes)
    updated = replace(updated, episode_id=episode_id(
        updated.seed, updated.level, updated.backend,
        updated.world_variant, updated.requested_colour))
    validate_episode(updated)
    return updated


# ── what the robot side is handed (stage C) ──────────────────────────────
def compat_mission_inputs(spec):
    """Return the ROS parameters the compatibility mission is given.

    Exactly two strings, and neither is a coordinate:

    - ``target_colour``: the task, from :meth:`EpisodeSpec.task_view`;
    - ``region_map``: :meth:`EpisodeSpec.region_map` in
      ``coco_config.robot.format_region_map``'s wire form, e.g.
      ``blue=lane_1,green=lane_4,red=lane_2,yellow=lane_3``.

    The second is the PRIVILEGED compatibility channel described on
    :meth:`EpisodeSpec.region_map`. The mission executive and ramp_driver
    resolve it to a lane through the static region table in coco_config;
    nothing on the robot side ever sees a target's x, y or z.
    """
    return {
        'target_colour': spec.task_view()['requested_colour'],
        'region_map': format_region_map(spec.region_map()),
    }


def resolve_episode(level='fixed', seed=0, requested_colour=None,
                    manifest_path='', backend='gazebo',
                    ramp_angle_deg=RAMP_ANGLE_DEG):
    """Return the episode a launch file was asked for.

    The one entry point both sides of a run call — the world launch that
    spawns it and the mission launch that is told about it — so the two
    cannot resolve an episode differently. Launch arguments arrive as
    strings; this accepts them as such.

    A non-empty `manifest_path` wins and is replayed exactly (it may be a
    hand-edited episode with no seed that regenerates it); it must still
    validate, and its backend must be `backend`. Otherwise the episode is
    generated from `seed` and `level`, with `requested_colour` pinned when
    given (the layout does not depend on it: it is drawn after every
    placement). `ramp_angle_deg` is the grade the world is built at; a
    generated episode rests its targets on that platform, and a replayed
    manifest must have been resolved for it (the backend checks).
    """
    if manifest_path:
        with open(manifest_path) as f:
            spec = episode_from_json(f.read())
        validate_episode(spec)
        if spec.backend != backend:
            raise InvalidEpisode(
                f'{manifest_path} is a {spec.backend} episode; this is the '
                f'{backend} backend')
        return spec
    try:
        seed = int(seed)
    except (TypeError, ValueError):
        raise InvalidEpisode(
            f'episode seed must be an integer, got {seed!r}') from None
    return generate_episode(seed=seed, level=str(level), backend=backend,
                            requested_colour=requested_colour or None,
                            ramp_angle_deg=ramp_angle_deg)


# ── results and reproducibility ──────────────────────────────────────────
#: What an episode can end as. ``void`` is a run that says nothing about
#: the robot — a harness fault, an orphaned simulator — and is recorded
#: rather than dropped, the same way the M6 matrices count void runs.
OUTCOMES = ('complete', 'failed', 'aborted', 'void')

#: Every timing key must name its clock. P0.2 learned this the hard way:
#: the executive's ``elapsed`` is sim time at a real-time factor near
#: 0.46, so an unlabelled duration is wrong by 2x depending on who reads
#: it. A key that does not end in one of these is refused.
CLOCK_SUFFIXES = ('_sim_s', '_wall_s')


@dataclass(frozen=True)
class EpisodeResult:
    """The machine-readable record of one run of one episode.

    It embeds the full privileged manifest the run used, so the record
    alone answers "what exactly happened in episode N?" and can replay
    it — even an episode that was hand-edited and has no seed that
    regenerates it. That makes a result an evaluation artefact: like the
    manifest, it is never an input to the robot.
    """

    episode_id: str
    outcome: str
    manifest: dict
    failure_reason: str = ''
    timings: dict = field(default_factory=dict)
    software_commit: str = ''
    policy_version: str = ''
    measurements: dict = field(default_factory=dict)

    def to_json(self, indent=2):
        """Serialise the record to a JSON string with sorted keys."""
        return json.dumps(asdict(self), indent=indent, sort_keys=True)

    def episode(self):
        """Rebuild the exact :class:`EpisodeSpec` this run used."""
        return episode_from_manifest(self.manifest)


def validate_result(result):
    """Raise :class:`InvalidEpisode` if `result` is not a usable record."""
    if result.outcome not in OUTCOMES:
        raise InvalidEpisode(
            f'unknown outcome {result.outcome!r}, expected one of {OUTCOMES}')
    if result.outcome == 'complete' and result.failure_reason:
        raise InvalidEpisode('a complete episode cannot carry a failure '
                             f'reason ({result.failure_reason!r})')
    if result.outcome != 'complete' and not result.failure_reason:
        raise InvalidEpisode(
            f'a {result.outcome} episode must say why (failure_reason)')
    if result.manifest.get('episode_id') != result.episode_id:
        raise InvalidEpisode(
            f'result {result.episode_id!r} embeds the manifest of '
            f'{result.manifest.get("episode_id")!r}')
    for key in result.timings:
        if not key.endswith(CLOCK_SUFFIXES):
            raise InvalidEpisode(
                f'timing {key!r} does not name its clock; end it in one '
                f'of {CLOCK_SUFFIXES}')
    validate_episode(result.episode())


def record_result(spec, outcome, failure_reason='', timings=None,
                  software_commit='', policy_version='', measurements=None):
    """Build and validate the :class:`EpisodeResult` of one run of `spec`."""
    result = EpisodeResult(
        episode_id=spec.episode_id,
        outcome=outcome,
        manifest=spec.manifest(),
        failure_reason=failure_reason,
        timings=dict(timings or {}),
        software_commit=software_commit,
        policy_version=policy_version,
        measurements=dict(measurements or {}),
    )
    validate_result(result)
    return result


def result_from_json(text):
    """Rebuild an :class:`EpisodeResult` from :meth:`EpisodeResult.to_json`."""
    data = json.loads(text)
    result = EpisodeResult(**data)
    validate_result(result)
    return result


def check_reproducible(result):
    """Regenerate `result`'s episode from its seed and compare manifests.

    Returns True when the seed still produces the recorded episode. False
    means the generator has changed since the run — the recorded result
    is still replayable through :meth:`EpisodeResult.episode`, but a new
    run of the same seed is no longer the same experiment, and a results
    table mixing the two would be comparing different tasks. That is the
    drift this hook exists to catch.
    """
    m = result.manifest
    regenerated = generate_episode(
        seed=m['seed'], level=m['level'], backend=m['backend'],
        world_variant=m['world_variant'],
        requested_colour=m['requested_colour'],
        ramp_angle_deg=m['ramp_angle_deg'],
        obstacles=result.episode().obstacles,
        metadata=m.get('metadata'))
    return regenerated.manifest() == result.episode().manifest()
