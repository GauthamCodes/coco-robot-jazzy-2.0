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
The Gazebo backend: an episode's targets as SDF models and spawn poses.

``full_world_robo.launch.py`` used to build the four targets itself, from
``coco_config.TARGETS``, as inline f-strings. It now asks this module,
with an :class:`~coco_sim.episode.EpisodeSpec`. For the default ``fixed``
episode the output is byte-for-byte what the launch file produced before
— ``test_backends.py`` holds a verbatim copy of the old code and asserts
it — so the P0.2 world is unchanged unless an episode is asked for.

Only the TARGETS come from the episode. The ramp, platform, down-slope
and arena are still built by the launch file from the same
``coco_config`` constants, because no episode level moves them.
"""

from dataclasses import dataclass

from coco_config.robot import target_by_colour

from ..episode import InvalidEpisode
from .common import SimulatorBackend


@dataclass(frozen=True)
class GazeboSpawn:
    """One ``ros_gz_sim create`` call: a model name, its SDF, a pose."""

    name: str
    sdf: str
    x: float
    y: float
    z: float

    def arguments(self):
        """The ``create`` argv, formatted exactly as the launch file did."""
        return ['-name', self.name, '-string', self.sdf,
                '-x', str(self.x), '-y', str(self.y), '-z', str(self.z)]


@dataclass(frozen=True)
class GazeboScene:
    """What the launch file needs to instantiate an episode's targets."""

    episode_id: str
    targets: tuple        # of GazeboSpawn, in manifest order
    magnet_models: tuple  # for magnet_release.py --models


def target_sdf(body):
    """Return the SDF document for one :class:`~.common.TargetBody`.

    Keep this text identical to what the launch file used to inline: a
    changed byte here is a changed world for every fixed-episode run, and
    the regression test compares against that original.
    """
    # coco_config's own string, verbatim: re-formatting the floats would
    # be one more place for a byte to change.
    rgb = target_by_colour(body.colour).rgb
    return f'''<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{body.name}">
    <link name="link">
      <inertial><mass>{body.mass}</mass>
        <inertia><ixx>{body.ixx:.6e}</ixx><iyy>{body.ixx:.6e}</iyy>
                 <izz>{body.izz:.6e}</izz>
                 <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
      <collision name="c"><geometry><cylinder>
        <radius>{body.radius}</radius><length>{body.height}</length></cylinder></geometry>
        <surface><friction><ode><mu>{body.friction}</mu><mu2>{body.friction}</mu2></ode></friction></surface>
      </collision>
      <visual name="v"><geometry><cylinder>
        <radius>{body.radius}</radius><length>{body.height}</length></cylinder></geometry>
        <material><ambient>{rgb} 1</ambient>
                  <diffuse>{rgb} 1</diffuse></material></visual>
    </link>
  </model>
</sdf>'''


class GazeboBackend(SimulatorBackend):
    """Translate an episode into Gazebo spawns."""

    name = 'gazebo'

    def translate(self, spec, ramp_angle_deg=None):
        """Return the :class:`GazeboScene` for `spec`.

        `ramp_angle_deg` is the grade the launch file is building. The
        manifest's target z assumes the grade the episode was resolved
        for, so a mismatch is refused rather than spawning targets inside
        or above the platform.
        """
        self.check(spec)
        if ramp_angle_deg is not None and \
                float(ramp_angle_deg) != float(spec.ramp_angle_deg):
            raise InvalidEpisode(
                f'episode {spec.episode_id} was resolved on a '
                f'{spec.ramp_angle_deg} deg ramp; the world is being built '
                f'at {ramp_angle_deg} deg')
        spawns = tuple(
            GazeboSpawn(name=body.name, sdf=target_sdf(body),
                        x=body.x, y=body.y, z=body.z)
            for body in self.bodies(spec))
        return GazeboScene(episode_id=spec.episode_id, targets=spawns,
                           magnet_models=tuple(s.name for s in spawns))
