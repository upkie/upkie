#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2022 Stéphane Caron
# Copyright 2023 Inria

import argparse
import asyncio
import logging
import os
import shutil
import tempfile
import time

import gin
import gymnasium as gym
import mpacklog
import numpy as np
from settings import EnvSettings
from stable_baselines3 import PPO

import upkie.envs
from upkie.envs import UpkieGroundVelocity
from upkie.utils.filters import low_pass_filter
from upkie.utils.log_path import new_log_filename
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


async def run_policy(
    env: UpkieGroundVelocity, policy, logger: mpacklog.AsyncLogger
) -> None:
    """
    Run policy, logging its actions and observations.

    Args:
        policy: Policy to run.
        logger: Logger to write actions and observations to.
    """
    action = np.zeros(env.action_space.shape)
    observation, info = env.reset()
    floor_contact = False
    while True:
        joystick = info["observation"].get("joystick", {})
        if joystick.get("cross_button", False):
            floor_contact = False

        action, _ = (
            policy.predict(observation)
            if floor_contact
            else no_contact_policy(action, env.dt)
        )
        action_time = time.time()
        observation, _, terminated, truncated, info = await env.async_step(
            action
        )
        floor_contact = info["observation"]["floor_contact"]["contact"]
        if terminated or truncated:
            observation, info = env.reset()
            floor_contact = False

        await logger.put(
            {
                "action": info["action"],
                "observation": info["observation"],
                "env": {
                    "rate": {
                        "slack": env.rate.slack,
                    }
                },
                "policy": {
                    "action": action,
                    "observation": observation,
                },
                "time": action_time,
            }
        )
    await logger.stop()


async def main(policy_path: str):
    settings = EnvSettings()
    with gym.make(
        "UpkieGroundVelocity-v1",
        frequency=settings.agent_frequency,
        max_ground_accel=settings.max_ground_accel,
        max_ground_velocity=settings.max_ground_velocity,
        regulate_frequency=True,
    ) as env:
        policy = PPO("MlpPolicy", env, verbose=1)
        policy.set_parameters(policy_path)
        logger = mpacklog.AsyncLogger("/dev/shm/ppo_balancer.mpack")
        await asyncio.gather(
            run_policy(env, policy, logger),
            logger.write(),
        )


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
        asyncio.run(main(policy_path))
    except KeyboardInterrupt:
        logging.info("Caught a keyboard interrupt")

    save_path = new_log_filename("ppo_balancer")
    shutil.copy("/dev/shm/ppo_balancer.mpack", save_path)
    logging.info(f"Log saved to {save_path}")
