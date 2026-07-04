"""
train_ppo.py
============
PPO training for Coco ramp traversal (Stable-Baselines3, CPU).

Usage (simulation must already be running, ideally headless):
  ros2 launch gazebo_models full_world_robo.launch.py gui:=false
  python3 -m coco_rl.train_ppo --steps 200000

A smoke test (--steps 1024) verifies the full loop in a couple of
minutes. Real training is an overnight job: wall-clock speed is bounded
by the simulator (RTF ~1), not by the tiny MLP policy — which is why the
CPU-only torch build is the right choice on this machine.
"""

import argparse

from stable_baselines3 import PPO
from stable_baselines3.common.monitor import Monitor

from coco_rl.ramp_env import CocoRampEnv


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--steps', type=int, default=200_000)
    ap.add_argument('--out', default='ppo_coco_ramp')
    args = ap.parse_args()

    env = Monitor(CocoRampEnv())
    model = PPO(
        'MlpPolicy', env,
        n_steps=512, batch_size=128,
        learning_rate=3e-4, gamma=0.99,
        policy_kwargs={'net_arch': [64, 64]},
        verbose=1, device='cpu',
    )
    try:
        model.learn(total_timesteps=args.steps, progress_bar=False)
        model.save(args.out)
        print(f'saved model -> {args.out}.zip')
    finally:
        env.close()


if __name__ == '__main__':
    main()
