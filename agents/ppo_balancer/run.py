#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2022 Stéphane Caron
# Copyright 2023 Inria
# SPDX-License-Identifier: Apache-2.0

import argparse
import logging
import os
import tempfile
from typing import Any, Optional, Tuple

import gin
import gymnasium as gym
import numpy as np
from gymnasium.wrappers import RescaleAction
from numpy.typing import NDArray
from settings import EnvSettings, PPOSettings
from stable_baselines3 import PPO

import upkie.envs
from upkie.envs.wrappers import DifferentiateAction
from upkie.utils.filters import low_pass_filter
from upkie.utils.raspi import configure_agent_process, on_raspi

upkie.envs.register()


def no_contact_policy(
    prev_action: NDArray[float], dt: float
) -> Tuple[NDArray[float], Any]:
    return (
        low_pass_filter(
            prev_action,
            cutoff_period=1.0,
            new_input=np.zeros(prev_action.shape),
            dt=dt,
        ),
        None,
    )


def run_policy(env, policy) -> None:
    """!
    Run policy in the robot environment.

    @param policy Policy to run.
    """
    action = np.zeros(env.action_space.shape)
    observation, info = env.reset()
    floor_contact = False
    reward = 0.0
    while True:
        joystick = info["observation"].get("joystick", {})
        if joystick.get("cross_button", False):
            floor_contact = False
        action, _ = (
            policy.predict(observation, deterministic=True)
            if floor_contact or True
            else no_contact_policy(action, env.dt)
        )
        env.log(
            {
                "action": action,
                "observation": observation,
                "reward": reward,
            }
        )
        observation, reward, terminated, truncated, info = env.step(action)
        floor_contact = info["observation"]["floor_contact"]["contact"]
        if terminated or truncated:
            observation, info = env.reset()
            floor_contact = False


def main(policy_path: str):
    env_settings = EnvSettings()
    with gym.make(
        env_settings.env_id,
        frequency=env_settings.agent_frequency,
        regulate_frequency=True,
        spine_config=env_settings.spine_config,
        max_ground_velocity=env_settings.max_ground_velocity,
    ) as velocity_env:
        accel_env = RescaleAction(
            DifferentiateAction(
                velocity_env,
                min_derivative=-env_settings.max_ground_accel,
                max_derivative=env_settings.max_ground_accel,
            ),
            min_action=-1.0,
            max_action=1.0,
        )
        ppo_settings = PPOSettings()
        policy = PPO(
            "MlpPolicy",
            accel_env,
            policy_kwargs={
                "net_arch": dict(
                    pi=ppo_settings.net_arch_pi,
                    vf=ppo_settings.net_arch_vf,
                ),
            },
            verbose=0,
        )
        policy.set_parameters(policy_path)
        run_policy(accel_env, policy)


def locate_policy(path: Optional[str], agent_dir: str) -> str:
    default_path = f"{agent_dir}/policy/params.zip"
    policy_path: str = default_path if path is None else path
    if not os.path.exists(policy_path):
        training_dir = f"{tempfile.gettempdir()}/ppo_balancer"
        if os.path.exists(f"{training_dir}/{policy_path}"):
            policy_path = f"{training_dir}/{policy_path}"
    if not os.path.exists(policy_path):
        raise FileNotFoundError(f"No policy params found at {policy_path=}")
    if policy_path.endswith(".zip"):
        policy_path = policy_path[:-4]
    return policy_path


if __name__ == "__main__":
    if on_raspi():
        configure_agent_process()

    agent_dir = os.path.abspath(os.path.dirname(__file__))
    gin.parse_config_file(f"{agent_dir}/settings.gin")

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "policy",
        nargs="?",
        help="path to the policy parameters file",
    )
    args = parser.parse_args()
    policy_path = locate_policy(args.policy, agent_dir)
    logging.info("Loading policy from %s.zip", policy_path)

    try:
        main(policy_path)
    except KeyboardInterrupt:
        logging.info("Caught a keyboard interrupt")
