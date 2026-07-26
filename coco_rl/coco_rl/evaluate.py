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
evaluate.py
===========
Deterministic evaluation of a trained ramp-traversal policy.

Usage (simulation must already be running, ideally headless):
  ros2 launch gazebo_models full_world_robo.launch.py gui:=false
  python3 -m coco_rl.evaluate ppo_model.zip --episodes 10 [--fast]
                                            [--randomize]

Runs the policy with deterministic actions and reports per-episode
outcome (goal / tipped / timeout), return, and length, plus a summary
success rate — the number that actually belongs in a README, as opposed
to a training-reward curve.
"""

import argparse

from stable_baselines3 import PPO

from coco_rl.ramp_env import CocoRampEnv
from coco_rl.train_ppo import set_physics


def run_episode(model, env):
    obs, _ = env.reset()
    total, steps = 0.0, 0
    while True:
        action, _ = model.predict(obs, deterministic=True)
        obs, reward, terminated, truncated, info = env.step(action)
        total += float(reward)
        steps += 1
        if terminated or truncated:
            # The env reports the outcome as fact. This used to be inferred
            # from the sign of the final reward, which quietly became wrong
            # whenever GOAL_BONUS or TIP_PENALTY was retuned.
            return info.get('outcome', 'unknown'), total, steps


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('model')
    ap.add_argument('--episodes', type=int, default=10)
    ap.add_argument('--fast', action='store_true')
    ap.add_argument('--randomize', action='store_true')
    ap.add_argument('--start-progress', type=float, default=0.0, metavar='M',
                    help='begin each episode this many metres along +x from '
                         'spawn, matching train_ppo --start-progress. Default 0 '
                         'is the FULL task. Note train_curriculum.sh does not '
                         'pass this, so a staged run evaluates every stage on '
                         'the full task even though the stage trained from '
                         'further along — that is the more meaningful number, '
                         'but it means an early stage legitimately scores low.')
    args = ap.parse_args()

    if args.fast:
        set_physics(0)
    env = CocoRampEnv(randomize=args.randomize,
                      start_progress=args.start_progress)
    model = PPO.load(args.model, device='cpu')
    outcomes = []
    try:
        for i in range(args.episodes):
            outcome, total, steps = run_episode(model, env)
            outcomes.append(outcome)
            print(f'episode {i + 1:2d}: {outcome:7s} '
                  f'return {total:7.2f}  steps {steps}')
    finally:
        env.close()
        if args.fast:
            set_physics(1.0)
    goals = outcomes.count('goal')
    summary = (f'\nsuccess rate: {goals}/{len(outcomes)} '
               f'({100 * goals / max(1, len(outcomes)):.0f}%)  '
               f"tipped: {outcomes.count('tipped')}  "
               f"timeout: {outcomes.count('timeout')}")
    # Surface anything that isn't a clean goal/tip/timeout rather than
    # letting it vanish from the tally.
    other = [o for o in outcomes if o not in ('goal', 'tipped', 'timeout')]
    if other:
        summary += f'  other: {len(other)} ({", ".join(sorted(set(other)))})'
    print(summary)


if __name__ == '__main__':
    main()
