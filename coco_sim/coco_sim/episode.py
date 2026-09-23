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
It resolves a specification and validates it. Emitting Gazebo SDF or an
Isaac USD stage from the same manifest is the next stage, and the point
of keeping this one backend-agnostic is that it can be.
"""

from dataclasses import asdict, dataclass, field, replace
import hashlib
import json
import math
import random

from coco_config.robot import (approach_stop_x, approach_window,
                               PLATFORM_LEN, RAMP_ANGLE_DEG, RAMP_RUN,
                               RAMP_SUMMIT_X, RAMP_WIDTH, SPAWN_XY, SPAWN_Z,
                               target_by_colour, TARGET_COLOURS,
                               TARGET_ROW_X, TARGETS, WHEEL_RADIUS,
                               WHEEL_SEPARATION, WHEEL_WIDTH, WHEELBASE)

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

#: World z of the platform's top surface. The wedge rises over RAMP_RUN
#: at RAMP_ANGLE_DEG and the platform sits flush with its crest.
PLATFORM_TOP_Z = RAMP_RUN * math.tan(math.radians(RAMP_ANGLE_DEG))

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

        expected_z = PLATFORM_TOP_Z + target.height / 2.0
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


# ── generation ───────────────────────────────────────────────────────────
def _fixed_layout():
    """Return the frozen P0.2 layout, straight out of ``coco_config``."""
    return [
        TargetSpec(colour=t.colour, model=t.model,
                   x=TARGET_ROW_X, y=t.lane_y,
                   z=PLATFORM_TOP_Z + t.height / 2.0,
                   diameter=t.diameter, height=t.height)
        for t in TARGETS
    ]


def _permuted_colours(rng):
    """Shuffle the four colours — level 1.

    Which *slot* a colour occupies changes; the set does not. That is
    enough on its own to break a compile-time colour->lane lookup, which
    is the whole point of the level.
    """
    colours = list(TARGET_COLOURS)
    rng.shuffle(colours)
    return colours


def _jittered(rng, diameter, base_y):
    """One (x, y) draw inside the validated envelope — level 2.

    Drawn inside ``PLACEMENT_FILL`` of the legal span rather than the
    whole of it, so a generated pose is never a rounding step from its
    own bound.
    """
    near, far = target_x_bounds(diameter)
    mid_x = (near + far) / 2.0
    half_x = (far - near) / 2.0 * PLACEMENT_FILL
    x = rng.uniform(mid_x - half_x, mid_x + half_x)

    y_low, y_high = target_y_bounds()
    span = (y_high - y_low) / len(TARGETS) / 2.0 * PLACEMENT_FILL
    y = rng.uniform(max(y_low, base_y - span), min(y_high, base_y + span))
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

    - ``'colours'`` permutes which colour occupies which lane;
    - ``'positions'`` permutes colours *and* jitters each target inside
      the validated manipulation envelope, by deterministic rejection
      sampling.

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

    if level == 'fixed':
        targets = _fixed_layout()
    else:
        colours = _permuted_colours(rng)
        lanes = [t.lane_y for t in TARGETS]
        targets = []
        for colour, lane in zip(colours, lanes):
            base = target_by_colour(colour)
            if level == 'colours':
                x, y = TARGET_ROW_X, lane
            else:
                x, y = _place(rng, base, lane, targets)
            targets.append(TargetSpec(
                colour=colour, model=base.model, x=x, y=y,
                z=PLATFORM_TOP_Z + base.height / 2.0,
                diameter=base.diameter, height=base.height))

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


def _place(rng, base, lane, placed):
    """Rejection-sample one legal (x, y) for `base`, or give up loudly.

    Deterministic: every draw comes from the episode's single generator,
    so the same seed rejects and accepts the same candidates in the same
    order.
    """
    for _ in range(MAX_PLACEMENT_ATTEMPTS):
        x, y = _jittered(rng, base.diameter, lane)
        candidate = TargetSpec(
            colour=base.colour, model=base.model, x=x, y=y,
            z=PLATFORM_TOP_Z + base.height / 2.0,
            diameter=base.diameter, height=base.height)
        if all(math.hypot(candidate.x - other.x, candidate.y - other.y)
               >= min_target_separation(candidate, other)
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
