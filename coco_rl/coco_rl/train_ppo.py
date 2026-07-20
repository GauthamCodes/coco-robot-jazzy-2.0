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
previous run (step counter and optimizer state included); --randomize
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
    ok = gz_service(
        f'/world/{WORLD}/set_physics', 'gz.msgs.Physics', 'gz.msgs.Boolean',
        f'max_step_size: 0.002, real_time_factor: {real_time_factor}')
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
    args = ap.parse_args()

    if args.fast:
        set_physics(0)   # unlimited — bounded by CPU, env steps on sim time

    env = Monitor(CocoRampEnv(randomize=args.randomize), filename=args.out)
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
            verbose=1, device='cpu',
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
