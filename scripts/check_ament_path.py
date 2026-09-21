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
Name every package on AMENT_PREFIX_PATH that is listed but cannot be found.

Why this exists
---------------
``ros_gz_sim``'s ``gz_sim.launch.py`` starts with
``GazeboRosPaths.get_paths()``, which walks EVERY package name on
``AMENT_PREFIX_PATH`` and resolves each one. The listing and the lookup
disagree about exactly one kind of entry: an ament index marker that is a
**dangling symlink**. ``get_resources()`` lists it (``os.listdir``, and it
is not a directory); ``get_resource()`` rejects it (``os.path.isfile`` is
False). One such marker anywhere on the path kills every ``gz sim`` launch
with ``package '<name>' not found`` -- naming a package COCO never asked
for.

A prefix with NO marker is harmless: it is never listed. What breaks is a
``--symlink-install`` marker whose build tree moved or went. Renaming the
workspace directory is enough. Measured on the development machine,
2026-09-22: ``<ws>/install`` put ``turtlebot3_teleop`` and ``red_ball_nav``
on the path, both markers pointing into the workspace's pre-rename
directory, and ``full_world_robo.launch.py`` died on the first of them.

Only reads. Prints one line per package to stderr and exits 1 when any
package is unresolvable, 0 otherwise. Pure standard library, so it runs
before anything is importable.
"""

import os
import sys

#: Where ament keeps one marker file per package, under each prefix.
MARKERS = os.path.join('share', 'ament_index', 'resource_index', 'packages')

TAG = '[setup_env]'


def find_unresolvable(prefix_path):
    """
    Return ``[(name, prefix, marker, target)]`` for each unresolvable package.

    Mirrors ``ament_index_python`` exactly: a name is LISTED from the first
    prefix whose index directory holds a non-directory, non-dot entry for
    it, and it RESOLVES if any prefix holds a regular file (or a symlink to
    one) of that name. ``target`` is the dangling symlink's target, or None.
    """
    prefixes = [p for p in prefix_path.split(os.pathsep) if p]
    listed = {}
    for prefix in prefixes:
        index = os.path.join(prefix, MARKERS)
        if not os.path.isdir(index):
            continue
        for name in sorted(os.listdir(index)):
            marker = os.path.join(index, name)
            if name.startswith('.') or os.path.isdir(marker):
                continue
            listed.setdefault(name, (prefix, marker))
    bad = []
    for name, (prefix, marker) in listed.items():
        if any(os.path.isfile(os.path.join(p, MARKERS, name))
               for p in prefixes):
            continue
        target = os.readlink(marker) if os.path.islink(marker) else None
        bad.append((name, prefix, marker, target))
    return bad


def main():
    """Report on the current AMENT_PREFIX_PATH; exit 1 if anything is bad."""
    bad = find_unresolvable(os.environ.get('AMENT_PREFIX_PATH', ''))
    if not bad:
        return 0
    err = sys.stderr
    print(f'{TAG} WARNING: {len(bad)} package(s) on AMENT_PREFIX_PATH are '
          'listed but cannot be resolved:', file=err)
    for name, prefix, _marker, target in bad:
        why = f'marker -> {target} (missing)' if target else 'marker unreadable'
        print(f'{TAG}   {name}  in {prefix}: {why}', file=err)
    print(f"{TAG} Every gz launch will die on one of them: \"package "
          f"'{bad[0][0]}' not found\". They are stale build products, not",
          file=err)
    print(f'{TAG} COCO dependencies. Build a COCO-only overlay and point '
          'COCO_WS at it:', file=err)
    print(f'{TAG}   scripts/build_overlay.sh "$HOME/coco_ws_build"', file=err)
    return 1


if __name__ == '__main__':
    sys.exit(main())
