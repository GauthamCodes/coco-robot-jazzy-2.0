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
train_ppo.py
============
PPO training for Coco ramp traversal (Stable-Baselines3, CPU).

Usage (simulation must already be running, ideally headless):
  ros2 launch gazebo_models full_world_robo.launch.py gui:=false
  python3 -m coco_rl.train_ppo --steps 200000 --fast

A smoke test (--steps 1024) verifies the full loop in a couple of
minutes. --fast unlocks the physics real-time factor for the duration of
the run (the env steps on sim time, so training simply runs quicker);
the previous cap is restored on exit. --resume model.zip continues a
previous run (step counter and optimizer state included) — this is also
how a curriculum phase transfers from the previous, steeper-by-a-notch
grade: relaunch the sim with ramp_angle:=<deg>, then train with
--resume <prev>.zip --ramp-angle <deg>. --randomize
varies the spawn lateral offset and yaw each episode. Episode
returns/lengths stream to <out>.monitor.csv for learning-curve plots,
and checkpoints land next to it every 25k steps. CPU torch is the right
choice here: the MLP policy is tiny and the simulator is the bottleneck.
"""

import argparse

from rclpy.executors import ExternalShutdownException
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.monitor import Monitor

from coco_rl.ramp_env import WORLD, CocoRampEnv, gz_service


def set_physics(real_time_factor):
    """Set the sim's real-time-factor cap at runtime (0 = unlimited).
    max_step_size must be re-sent or the UserCommands system would treat
    the proto default (0) as 'unset'."""
    # Retried for the same reason as the set_pose in ramp_env.reset(): a
    # transient gz-transport miss here is not fatal, but it silently costs
    # ~4x throughput for the whole run by leaving the RTF cap in place.
    ok = gz_service(
        f'/world/{WORLD}/set_physics', 'gz.msgs.Physics', 'gz.msgs.Boolean',
        f'max_step_size: 0.002, real_time_factor: {real_time_factor}',
        attempts=3)
    print(f'set_physics(rtf={real_time_factor}): '
          f'{"ok" if ok else "FAILED — is the sim running?"}')
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--steps', type=int, default=200_000)
    ap.add_argument('--out', default='ppo_coco_ramp')
    ap.add_argument('--fast', action='store_true',
                    help='unlock the physics real-time factor while training')
    ap.add_argument('--resume', default=None, metavar='MODEL_ZIP',
                    help='continue training from a saved model')
    ap.add_argument('--randomize', action='store_true',
                    help='randomize spawn lateral offset and yaw each episode')
    ap.add_argument('--ramp-angle', type=int, default=None, metavar='DEG',
                    help='curriculum grade of the ramp this run trains against '
                         '(deg). Recorded in the run header only — the grade is '
                         'set by the running sim (launch ramp_angle:=<deg>), not '
                         'here. Combine with --resume to transfer across phases.')
    ap.add_argument('--seed', type=int, default=0,
                    help='RNG seed for PPO and the env reset (default 0). '
                         'Recorded in the run header; note the simulator '
                         'itself is not bit-reproducible, so this pins the '
                         'policy and sampling, not the physics.')
    args = ap.parse_args()

    if args.fast:
        set_physics(0)   # unlimited — bounded by CPU, env steps on sim time

    # Recorded so a run can be reproduced from its log. PPO(seed=...) seeds
    # python/numpy/torch and the action space, and hands the seed to the
    # VecEnv, which applies it at the first reset — so no extra reset (and
    # no extra robot teleport) is needed here.
    print(f'seed={args.seed} steps={args.steps} randomize={args.randomize} '
          f'ramp_angle={args.ramp_angle}')

    # info_keywords=('outcome',) adds an `outcome` column to the Monitor CSV.
    # Without it the CSV holds only (return, length, time), and every episode
    # that ended before max_steps looks identical — so a tip-over, a
    # `sim_stalled` truncation and a goal are indistinguishable after the fact.
    # A 180k-step run was analysed by *inferring* outcomes from return and
    # length, which cannot separate a tip-over (−10 penalty) from a stall
    # (reward 0.0), and produced a confidently wrong diagnosis. The env already
    # reports the outcome as fact; log it.
    env = Monitor(CocoRampEnv(randomize=args.randomize), filename=args.out,
                  info_keywords=('outcome',))
    if args.resume:
        model = PPO.load(args.resume, env=env, device='cpu')
        print(f'resumed from {args.resume} '
              f'(prior timesteps: {model.num_timesteps})')
    else:
        model = PPO(
            'MlpPolicy', env,
            n_steps=512, batch_size=128,
            learning_rate=3e-4, gamma=0.99,
            policy_kwargs={'net_arch': [64, 64]},
            verbose=1, device='cpu', seed=args.seed,
        )
    checkpoints = CheckpointCallback(
        save_freq=25_000, save_path='.', name_prefix=args.out)
    try:
        model.learn(total_timesteps=args.steps, progress_bar=False,
                    callback=checkpoints,
                    reset_num_timesteps=args.resume is None)
        model.save(args.out)
        print(f'saved model -> {args.out}.zip  (episodes: {args.out}.monitor.csv)')
    except (KeyboardInterrupt, ExternalShutdownException):
        # A long run is hours of wall clock; do not throw it away on Ctrl-C.
        # ExternalShutdownException is the one that actually fires: rclpy
        # installs its own SIGINT handler and invalidates the context, so
        # the spin inside env.step() raises that rather than KeyboardInterrupt.
        # Saved under a distinct name so it can never clobber a completed run.
        path = f'{args.out}_interrupted'
        model.save(path)
        print(f'\ninterrupted at {model.num_timesteps} timesteps — '
              f'saved -> {path}.zip\n'
              f'resume with: --resume {path}.zip --steps <total>')
        raise SystemExit(130)
    finally:
        env.close()
        if args.fast:
            set_physics(1.0)


if __name__ == '__main__':
    main()
