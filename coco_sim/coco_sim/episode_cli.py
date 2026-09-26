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

r"""
coco_episode — make, inspect, check and record episodes from the shell.

Installed on PATH as ``coco_episode`` (install/coco_sim/bin, like write_mjcf)::

    # an episode from a seed, as the manifest both launch files replay
    coco_episode generate --level colours --seed 1 --colour yellow \
        --out manifest.json
    # what the robot side is told about it (names only)
    coco_episode inputs --manifest manifest.json
    # did the running gz build what the manifest says?
    coco_episode readback --manifest manifest.json
    # does this seed still generate the recorded episode?
    coco_episode check --manifest manifest.json
    # the machine-readable result of one run
    coco_episode result --manifest manifest.json --outcome complete \
        --timing mission_sim_s=170.4 --out result.json

No ROS import: ``readback`` shells out to the ``gz`` CLI, the same way
``coco_perception.vision_check`` reads ground truth.
"""

import argparse
import json
import sys

from .backends import check_instantiation
from .backends.gazebo import read_gz_poses
from .episode import (compat_mission_inputs, episode_from_json,
                      generate_episode, InvalidEpisode, LEVELS, OUTCOMES,
                      record_result, validate_episode)


def _load(path):
    with open(path) as f:
        spec = episode_from_json(f.read())
    validate_episode(spec)
    return spec


def _emit(text, out):
    if out:
        with open(out, 'w') as f:
            f.write(text + '\n')
    else:
        print(text)


def cmd_generate(args):
    """Print (or write) the manifest of a generated episode."""
    spec = generate_episode(seed=args.seed, level=args.level,
                            backend=args.backend,
                            requested_colour=args.colour or None)
    _emit(spec.to_json(), args.out)
    return 0


def cmd_inputs(args):
    """Print exactly what the compatibility mission is told."""
    _emit(json.dumps(compat_mission_inputs(_load(args.manifest)),
                     indent=2, sort_keys=True), args.out)
    return 0


def cmd_readback(args):
    """Compare a running gz's targets with the manifest."""
    spec = _load(args.manifest)
    poses = read_gz_poses([t.model for t in spec.targets])
    report = check_instantiation(spec, poses, xy_tol=args.xy_tol,
                                 z_tol=args.z_tol, tilt_tol=args.tilt_tol)
    report['episode_id'] = spec.episode_id
    report['tolerances'] = {'xy_m': args.xy_tol, 'z_m': args.z_tol,
                            'tilt_rad': args.tilt_tol}
    report['observed'] = {name: vars(pose) for name, pose in poses.items()}
    _emit(json.dumps(report, indent=2, sort_keys=True), args.out)
    return 0 if report['ok'] else 1


def cmd_check(args):
    """Say whether the seed still regenerates the recorded episode."""
    spec = _load(args.manifest)
    again = generate_episode(
        seed=spec.seed, level=spec.level, backend=spec.backend,
        world_variant=spec.world_variant,
        requested_colour=spec.requested_colour,
        ramp_angle_deg=spec.ramp_angle_deg, obstacles=spec.obstacles,
        metadata=spec.metadata)
    same = again.to_json() == spec.to_json()
    print(f'{spec.episode_id}: {"reproducible" if same else "DRIFTED"}')
    return 0 if same else 1


def cmd_result(args):
    """Write the validated EpisodeResult of one run."""
    spec = _load(args.manifest)
    timings = {}
    for item in args.timing or []:
        key, _, value = item.partition('=')
        timings[key] = float(value)
    measurements = json.loads(args.measurements) if args.measurements else {}
    result = record_result(
        spec, args.outcome, failure_reason=args.reason or '',
        timings=timings, software_commit=args.commit or '',
        policy_version=args.policy or '', measurements=measurements)
    _emit(result.to_json(), args.out)
    return 0


def build_parser():
    """Return the argparse parser for every subcommand."""
    parser = argparse.ArgumentParser(prog='coco_episode',
                                     description=__doc__.split('\n')[1])
    sub = parser.add_subparsers(dest='command', required=True)

    gen = sub.add_parser('generate', help=cmd_generate.__doc__)
    gen.add_argument('--level', choices=LEVELS, default='fixed')
    gen.add_argument('--seed', type=int, default=0)
    gen.add_argument('--colour', default='')
    gen.add_argument('--backend', default='gazebo')
    gen.add_argument('--out', default='')
    gen.set_defaults(func=cmd_generate)

    for name, func in (('inputs', cmd_inputs), ('check', cmd_check)):
        p = sub.add_parser(name, help=func.__doc__)
        p.add_argument('--manifest', required=True)
        p.add_argument('--out', default='')
        p.set_defaults(func=func)

    back = sub.add_parser('readback', help=cmd_readback.__doc__)
    back.add_argument('--manifest', required=True)
    back.add_argument('--xy-tol', type=float, default=0.005)
    back.add_argument('--z-tol', type=float, default=0.005)
    back.add_argument('--tilt-tol', type=float, default=0.05)
    back.add_argument('--out', default='')
    back.set_defaults(func=cmd_readback)

    res = sub.add_parser('result', help=cmd_result.__doc__)
    res.add_argument('--manifest', required=True)
    res.add_argument('--outcome', choices=OUTCOMES, required=True)
    res.add_argument('--reason', default='')
    res.add_argument('--timing', action='append',
                     help='KEY=SECONDS; KEY must end _sim_s or _wall_s')
    res.add_argument('--commit', default='')
    res.add_argument('--policy', default='')
    res.add_argument('--measurements', default='',
                     help='a JSON object of extra measurements')
    res.add_argument('--out', default='')
    res.set_defaults(func=cmd_result)
    return parser


def main(argv=None):
    """Entry point: ``coco_episode <command> ...``."""
    args = build_parser().parse_args(argv)
    try:
        return args.func(args)
    except InvalidEpisode as exc:
        print(f'coco_episode: invalid episode: {exc}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    sys.exit(main())
