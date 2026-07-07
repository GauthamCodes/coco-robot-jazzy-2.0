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
        obs, reward, terminated, truncated, _ = env.step(action)
        total += float(reward)
        steps += 1
        if terminated or truncated:
            # terminated == goal or tip; disambiguate via the reward sign
            # of the final step (goal bonus is strongly positive)
            if not terminated:
                outcome = 'timeout'
            elif reward > 0:
                outcome = 'goal'
            else:
                outcome = 'tipped'
            return outcome, total, steps


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('model')
    ap.add_argument('--episodes', type=int, default=10)
    ap.add_argument('--fast', action='store_true')
    ap.add_argument('--randomize', action='store_true')
    args = ap.parse_args()

    if args.fast:
        set_physics(0)
    env = CocoRampEnv(randomize=args.randomize)
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
    print(f'\nsuccess rate: {goals}/{len(outcomes)} '
          f'({100 * goals / max(1, len(outcomes)):.0f}%)  '
          f"tipped: {outcomes.count('tipped')}  "
          f"timeout: {outcomes.count('timeout')}")


if __name__ == '__main__':
    main()
