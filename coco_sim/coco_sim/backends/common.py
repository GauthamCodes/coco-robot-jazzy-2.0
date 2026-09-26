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
The engine-neutral half of a simulator backend.

Everything a backend needs to know about an episode's targets that is
NOT specific to one engine is computed here, once: the physical body
(mass, inertia, friction, colour) and the world pose, both taken from
the manifest and ``coco_config``. A backend then only translates a
:class:`TargetBody` into its own object — an SDF model, a USD prim.

If this layer did not exist, each backend would re-derive inertia and
re-type the friction coefficient, and the first thing the Gazebo/Isaac
comparison would measure is the difference between two hand-copied
constants. That is CLAUDE.md rule 3's argument, one level up.

Pure stdlib + ``coco_config`` + :mod:`coco_sim.episode`: no ROS, no gz,
no Isaac. Importable anywhere a manifest is.
"""

from dataclasses import dataclass, replace
import math

from coco_config.robot import target_by_colour, TARGET_MASS

from ..episode import InvalidEpisode, TargetSpec, validate_episode

#: Coulomb friction of every target's contact surface. It was a literal
#: in full_world_robo.launch.py's inline SDF (``<mu>1.5</mu>``); it is
#: defined once here so every backend builds the same object.
TARGET_FRICTION = 1.5


@dataclass(frozen=True)
class TargetBody:
    """One target as a rigid body in the world, engine-neutral.

    Built from a :class:`~coco_sim.episode.TargetSpec` and nothing else
    but ``coco_config``. ``name`` is the model name the magnet binds and
    the grasp checks by — the object's identity, which travels with its
    colour wherever the episode puts it.
    """

    name: str
    colour: str
    rgb: tuple            # (r, g, b) in [0, 1]
    radius: float
    height: float
    mass: float
    ixx: float            # = iyy; a solid cylinder about its centre
    izz: float
    friction: float
    x: float
    y: float
    z: float              # centre of mass, resting on the platform


def target_body(target):
    """Return the :class:`TargetBody` of one manifest target."""
    frozen = target_by_colour(target.colour)
    radius = target.diameter / 2.0
    height = target.height
    # Solid cylinder about its centre of mass — the launch file's formula,
    # moved here verbatim so every backend inherits the same numbers.
    ixx = TARGET_MASS * (3.0 * radius ** 2 + height ** 2) / 12.0
    izz = TARGET_MASS * radius ** 2 / 2.0
    return TargetBody(
        name=target.model, colour=target.colour,
        rgb=tuple(float(v) for v in frozen.rgb.split()),
        radius=radius, height=height, mass=TARGET_MASS,
        ixx=ixx, izz=izz, friction=TARGET_FRICTION,
        x=target.x, y=target.y, z=target.z)


class SimulatorBackend:
    """Translate one :class:`~coco_sim.episode.EpisodeSpec` for one engine.

    The contract every backend keeps:

    - it consumes the manifest as it is — it never re-derives a pose,
      picks a target, or draws a random number;
    - it refuses an episode recorded for another engine, because the
      manifest's ``backend`` field is what attributes a result;
    - it re-validates, so a hand-edited manifest cannot reach a
      simulator without passing the same envelope the generator does.

    Subclasses set :attr:`name` and implement :meth:`translate`.
    """

    name = ''

    def check(self, spec):
        """Raise :class:`InvalidEpisode` unless `spec` is ours and legal."""
        if spec.backend != self.name:
            raise InvalidEpisode(
                f'episode {spec.episode_id} was resolved for the '
                f'{spec.backend} backend, not {self.name}')
        validate_episode(spec)

    def bodies(self, spec):
        """Return every target of `spec` as a :class:`TargetBody`."""
        return tuple(target_body(t) for t in spec.targets)

    def translate(self, spec, **engine):
        """Return this engine's description of `spec`. Subclass hook."""
        raise NotImplementedError


# ── read-back: did the simulator build what the manifest says? ───────────
@dataclass(frozen=True)
class ObservedPose:
    """A target pose read back from a running simulator."""

    x: float
    y: float
    z: float
    roll: float = 0.0
    pitch: float = 0.0


def check_instantiation(spec, observed, xy_tol=0.005, z_tol=0.005,
                        tilt_tol=0.05):
    """Compare a simulator's actual targets with the manifest.

    `observed` maps model name -> :class:`ObservedPose`. Returns a report
    dict: per model the error in x, y, z and tilt and whether each is in
    tolerance, and ``'layout_valid'`` — the manifest's own
    :func:`~coco_sim.episode.validate_episode` re-run on the OBSERVED
    x, y, which is what catches a target that settled somewhere legal
    for its pose check but into another's approach corridor.

    Engine-neutral: gz, Isaac and MuJoCo read-backs all reduce to this.
    Tolerances are arguments, not measured constants; the caller states
    the ones it used in its evidence.
    """
    report = {'models': {}, 'missing': [], 'layout_valid': False,
              'layout_error': ''}
    moved = []
    for target in spec.targets:
        pose = observed.get(target.model)
        if pose is None:
            report['missing'].append(target.model)
            continue
        tilt = math.hypot(pose.roll, pose.pitch)
        entry = {
            'dx': pose.x - target.x, 'dy': pose.y - target.y,
            'dz': pose.z - target.z, 'tilt': tilt,
        }
        entry['xy_ok'] = math.hypot(entry['dx'], entry['dy']) <= xy_tol
        entry['z_ok'] = abs(entry['dz']) <= z_tol
        entry['upright'] = tilt <= tilt_tol
        entry['ok'] = entry['xy_ok'] and entry['z_ok'] and entry['upright']
        report['models'][target.model] = entry
        # Keep the manifest z: settling moves z by contact compliance, and
        # the envelope's z rule is about the spec, not the physics.
        moved.append(TargetSpec(
            colour=target.colour, model=target.model, x=pose.x, y=pose.y,
            z=target.z, diameter=target.diameter, height=target.height,
            region_id=target.region_id))
    if not report['missing']:
        try:
            validate_episode(replace(spec, targets=tuple(moved)))
            report['layout_valid'] = True
        except InvalidEpisode as exc:
            report['layout_error'] = str(exc)
    report['ok'] = (report['layout_valid'] and not report['missing']
                    and all(m['ok'] for m in report['models'].values()))
    return report
