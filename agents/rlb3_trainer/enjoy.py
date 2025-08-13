#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2025 Inria

import argparse
from pathlib import Path
from typing import Optional

import gymnasium as gym
from stable_baselines3 import PPO

import upkie.envs
import upkie.logging
from agents.rlb3_trainer.select_env import ENVIRONMENTS, select_env


def find_latest_model(
    logs_dir: Path, env_name: str, algo: str = "ppo"
) -> Optional[Path]:
    """
    Find the latest trained model for the given environment.

    Args:
        logs_dir: Directory where models are stored
        env_name: Environment name
        algo: Algorithm name

    Returns:
        Path to the model file, or None if not found
    """
    algo_dir = logs_dir / algo
    if not algo_dir.exists():
        return None

    env_dirs = [
        file
        for file in algo_dir.iterdir()
        if file.is_dir() and env_name in file.name
    ]
    if not env_dirs:
        return None

    latest_dir = max(env_dirs, key=lambda x: x.stat().st_mtime)
    model_file = latest_dir / f"{env_name}.zip"
    if model_file.exists():
        return model_file

    return None


def parse_command_line_arguments() -> argparse.Namespace:
    """
    Parse command line arguments.

    Returns:
        Command-line arguments.
    """
    parser = argparse.ArgumentParser(
        description="Run trained RL policy on Upkie environment"
    )
    parser.add_argument(
        "--model-path",
        help="Path to trained model file",
        default="",
        type=str,
    )
    parser.add_argument(
        "--env",
        help="Environment ID",
        default="",
        type=str,
    )
    parser.add_argument(
        "--algo",
        help="RL Algorithm",
        default="ppo",
        type=str,
    )
    parser.add_argument(
        "--logs-dir",
        help="Logs directory",
        default="./logs",
        type=str,
    )
    parser.add_argument(
        "--episodes",
        help="Number of episodes to run",
        default=5,
        type=int,
    )
    return parser.parse_args()


def get_environment_name(env_arg: Optional[str] = None) -> str:
    """
    Get environment name, prompting user if not provided.

    Args:
        env_arg: Environment argument from command line

    Returns:
        Selected environment name
    """
    if env_arg is None:
        return select_env()
    elif env_arg in ENVIRONMENTS:
        return env_arg
    else:
        print(f"Error: Unknown environment '{env_arg}'")
        print("Available environments:")
        for env in ENVIRONMENTS:
            print(f"  - {env}")
        return select_env()


def main():
    """Main function to run trained policy."""
    args = parse_command_line_arguments()
    upkie.envs.register()
    upkie.logging.disable_warnings()

    # Get environment name with interactive selection if not provided
    env_name = get_environment_name(args.env if args.env else None)

    # Find model path
    if args.model_path:
        model_path = Path(args.model_path)
    else:
        model_path = find_latest_model(
            Path(args.logs_dir), env_name, args.algo
        )

    if model_path is None or not model_path.exists():
        print(f"Error: Could not find trained model for {env_name}")
        print(f"Searched in: {args.logs_dir}/{args.algo}/")
        print("Train a model first using: pixi run rlb3-train-pybullet")
        return

    print(f"Loading model from: {model_path}")

    # Load the trained model
    if args.algo.lower() == "ppo":
        model = PPO.load(model_path)
    else:
        print(f"Error: Algorithm {args.algo} not supported yet")
        return

    # Create environment and run episodes
    with gym.make(env_name, gui=True, frequency=200) as env:
        print(f"Running {args.episodes} episodes")
        for episode in range(args.episodes):
            obs, info = env.reset()
            episode_reward = 0
            steps = 0
            done = False

            print(f"\nEpisode {episode + 1}/{args.episodes}")

            while not done:
                action, _ = model.predict(obs, deterministic=True)
                obs, reward, terminated, truncated, info = env.step(action)
                done = terminated or truncated
                episode_reward += reward
                steps += 1

                if steps % 100 == 0:
                    print(f"  Step {steps}, Reward: {episode_reward:.2f}")

            print(
                f"  Episode finished - "
                f"Total reward: {episode_reward:.2f}, "
                f"Steps: {steps}"
            )


if __name__ == "__main__":
    main()
