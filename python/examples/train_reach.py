"""Teach a robot arm to reach a point with augmented random search (a linear policy).

Runs headless and fast-forward: leave it overnight with --iterations 2000. Only numpy is
needed; to train with PyTorch or JAX instead, keep the VecEnv calls and swap the policy
update for your optimiser (torch.as_tensor(obs) / jax.numpy.asarray(obs)).

    python3 python/examples/train_reach.py --iterations 30
"""

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from sicdm_env import VecEnv  # noqa: E402

# A three-joint arm: turret, shoulder and elbow, the forearm its tool.
ARM = [
    "BOX 100 100 100 @0,0,0",
    "BOX 80 80 60 @0,0,100",
    "BOX 40 40 300 @0,0,160",
    "BOX 40 40 300 @0,0,460",
    'JOINT "Block 2" TO "Block 1" AT 0,0,100 AXIS Z RANGE -180 180',
    'JOINT "Block 3" TO "Block 2" AT 0,0,160 AXIS Y RANGE -120 120',
    'JOINT "Block 4" TO "Block 3" AT 0,0,460 AXIS Y RANGE -150 150',
    'RIG TAG "Block 4" END_EFFECTOR',
]


def rollout(env, policies, steps):
    """Runs one policy per copy for `steps` actions; returns each copy's total reward."""
    obs = env.reset()
    total = np.zeros(env.num_envs)
    for _ in range(steps):
        x = np.concatenate([obs, np.ones((env.num_envs, 1))], axis=1)  # with a bias
        actions = np.clip(np.einsum("nao,no->na", policies, x), -1, 1)
        obs, reward, _, _, _ = env.step(actions)
        total += reward
    return total


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--iterations", type=int, default=30)
    ap.add_argument("--directions", type=int, default=8)
    ap.add_argument("--steps", type=int, default=50)
    ap.add_argument("--step-size", type=float, default=0.02)
    ap.add_argument("--explore", type=float, default=0.05)
    ap.add_argument("--project", default=None, help="a .sicdm with the robot instead of the built-in arm")
    args = ap.parse_args()

    count = 2 * args.directions
    with VecEnv(project=args.project, setup=() if args.project else ARM, count=count, task="reach", target=(300, 200, 300),
                episode_seconds=args.steps * 0.02 + 1, noise=0.0, seed=7) as env:
        theta = np.zeros((env.action_size, env.observation_size + 1))
        rng = np.random.default_rng(0)
        for it in range(args.iterations):
            delta = rng.standard_normal((args.directions,) + theta.shape)
            policies = np.concatenate([theta + args.explore * delta, theta - args.explore * delta])
            returns = rollout(env, policies, args.steps)
            plus, minus = returns[: args.directions], returns[args.directions:]
            sigma = returns.std() + 1e-8
            theta += args.step_size / (args.directions * sigma) * np.einsum("d,dao->ao", plus - minus, delta)
            base = rollout(env, np.repeat(theta[None], count, axis=0), args.steps)[0]
            print(f"iteration {it + 1:4d}: return {base:8.3f}  (tool {-base / args.steps * 1000:6.1f} mm from the target on average)")


if __name__ == "__main__":
    main()
