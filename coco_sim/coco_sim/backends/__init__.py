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
Simulator backends: one episode manifest, translated per engine.

::

    EpisodeSpec (coco_sim.episode)         the semantics, once
         │
         ├── GazeboBackend.translate()     SDF models + spawn poses
         └── IsaacBackend.translate()      USD primitive specs (data only)

A backend never decides anything about the episode. It does not draw a
number, choose a target or move one; it turns :class:`TargetSpec` into
its engine's object and refuses a manifest recorded for another engine.
MuJoCo is a valid ``EpisodeSpec.backend`` but has no adapter: the MuJoCo
envs are the RL climb and carry no fetch targets.
"""

from .common import (check_instantiation, ObservedPose, SimulatorBackend,
                     target_body, TargetBody)
from .gazebo import GazeboBackend, GazeboScene, GazeboSpawn
from .isaac import IsaacBackend, IsaacPrim, IsaacScene

#: The engines with an adapter, by the name ``EpisodeSpec.backend`` uses.
ADAPTERS = {
    GazeboBackend.name: GazeboBackend,
    IsaacBackend.name: IsaacBackend,
}


def backend_for(spec):
    """Return the adapter instance for `spec`'s backend, or raise KeyError."""
    return ADAPTERS[spec.backend]()


__all__ = [
    'ADAPTERS', 'backend_for', 'check_instantiation', 'GazeboBackend',
    'GazeboScene', 'GazeboSpawn', 'IsaacBackend', 'IsaacPrim', 'IsaacScene',
    'ObservedPose', 'SimulatorBackend', 'target_body', 'TargetBody',
]
