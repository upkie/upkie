#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2022 Stéphane Caron
# Copyright 2023 Inria
# SPDX-License-Identifier: Apache-2.0

import argparse
import logging
import os

import gin
import gymnasium as gym
import numpy as np
from envs import make_ppo_balancer_env
from settings import EnvSettings, PPOSettings
from stable_baselines3 import PPO

import upkie.envs
from upkie.utils.raspi import configure_agent_process, on_raspi

upkie.envs.register()


def parse_command_line_arguments() -> argparse.Namespace:
    """
    Parse command line arguments.

    Returns:
        Command-line arguments.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "policy",
        nargs="?",
        help="path to the policy parameters file",
    )
    parser.add_argument(
        "--training",
        default=False,
        action="store_true",
        help="add noise and delays as in training",
    )
    return parser.parse_args()


def get_tip_state(observation):
    pitch = observation[0]
    ground_position = observation[1]
    angular_velocity = observation[2]
    ground_velocity = observation[3]

    tip_height = 0.58  # [m]
    tip_position = ground_position + tip_height * np.sin(pitch)
    tip_velocity = ground_velocity + tip_height * angular_velocity * np.cos(
        pitch
    )
    return tip_position, tip_velocity


def run_policy(env, policy) -> None:
    """!
    Run policy in the robot environment.

    @param policy Policy to run.
    """
    action = np.zeros(env.action_space.shape)
    observation, info = env.reset()
    reward = 0.0
    while True:
        action, _ = policy.predict(observation, deterministic=True)
        tip_position, tip_velocity = get_tip_state(observation[-1])
        env.log(
            {
                "action": action,
                "observation": observation[-1],
                "reward": reward,
                "tip_position": tip_position,
                "tip_velocity": tip_velocity,
            }
        )
        observation, reward, terminated, truncated, info = env.step(action)
        if terminated or truncated:
            observation, info = env.reset()


def main(policy_path: str, training: bool):
    env_settings = EnvSettings()
    with gym.make(
        env_settings.env_id,
        frequency=env_settings.agent_frequency,
        regulate_frequency=True,
        spine_config=env_settings.spine_config,
        max_ground_velocity=env_settings.max_ground_velocity,
    ) as velocity_env:
        env = make_ppo_balancer_env(
            velocity_env,
            env_settings,
            training=training,
        )
        ppo_settings = PPOSettings()
        policy = PPO(
            "MlpPolicy",
            env,
            policy_kwargs={
                "net_arch": dict(
                    pi=ppo_settings.net_arch_pi,
                    vf=ppo_settings.net_arch_vf,
                ),
            },
            verbose=0,
        )
        policy.set_parameters(policy_path)
        run_policy(env, policy)


if __name__ == "__main__":
    if on_raspi():
        configure_agent_process()

    agent_dir = os.path.abspath(os.path.dirname(__file__))
    gin.parse_config_file(f"{agent_dir}/policy/operative_config.gin")

    args = parse_command_line_arguments()
    policy_path = args.policy
    if policy_path is None:
        policy_path = f"{agent_dir}/policy/params.zip"
    if policy_path.endswith(".zip"):
        policy_path = policy_path[:-4]
    logging.info("Loading policy from %s.zip", policy_path)

    try:
        main(policy_path, args.training)
    except KeyboardInterrupt:
        logging.info("Caught a keyboard interrupt")
