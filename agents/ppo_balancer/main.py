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

import gin
import gymnasium as gym
import numpy as np
from settings import EnvSettings, PPOSettings
from stable_baselines3 import PPO

import upkie.envs
from upkie.envs import UpkieGroundVelocity
from upkie.utils.filters import low_pass_filter
from upkie.utils.raspi import configure_agent_process, on_raspi

upkie.envs.register()


def no_contact_policy(prev_action: np.ndarray, dt: float) -> np.ndarray:
    return (
        low_pass_filter(
            prev_action,
            cutoff_period=1.0,
            new_input=np.zeros(prev_action.shape),
            dt=dt,
        ),
        None,
    )


def run_policy(env: UpkieGroundVelocity, policy) -> None:
    """
    Run policy in the robot environment.

    Args:
        policy: Policy to run.
    """
    action = np.zeros(env.action_space.shape)
    observation, info = env.reset()
    floor_contact = False
    while True:
        joystick = info["observation"].get("joystick", {})
        if joystick.get("cross_button", False):
            floor_contact = False
        action, _ = (
            policy.predict(observation, deterministic=True)
            if floor_contact
            else no_contact_policy(action, env.dt)
        )
        action[0] = 1.0 * action[0]
        observation, _, terminated, truncated, info = env.step(action)
        floor_contact = info["observation"]["floor_contact"]["contact"]
        if terminated or truncated:
            observation, info = env.reset()
            floor_contact = False


def main(policy_path: str):
    settings = EnvSettings()
    with gym.make(
        settings.env_id,
        frequency=settings.agent_frequency,
        max_ground_accel=settings.max_ground_accel,
        max_ground_velocity=settings.max_ground_velocity,
        regulate_frequency=True,
        spine_config={
            "bullet": {
                "torque_control": {
                    "kd": settings.sim_torque_control_kd,
                },
            },
        },
        # velocity_filter=None,
    ) as env:
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
            verbose=1,
        )
        policy.set_parameters(policy_path)
        run_policy(env, policy)


if __name__ == "__main__":
    if on_raspi():
        configure_agent_process()

    agent_dir = os.path.abspath(os.path.dirname(__file__))
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "policy",
        nargs="?",
        help="path to the policy parameters file",
    )
    args = parser.parse_args()
    if args.policy is None:
        args.policy = f"{agent_dir}/policy/params.zip"
    gin.parse_config_file(f"{agent_dir}/settings.gin")

    policy_path = args.policy
    training_dir = f"{tempfile.gettempdir()}/ppo_balancer"
    if policy_path.endswith(".zip"):
        policy_path = policy_path[:-4]
    if os.path.exists(f"{training_dir}/{policy_path}.zip"):
        policy_path = f"{training_dir}/{policy_path}"

    try:
        main(policy_path)
    except KeyboardInterrupt:
        logging.info("Caught a keyboard interrupt")
