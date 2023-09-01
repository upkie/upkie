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
import mpacklog
import numpy as np
from loop_rate_limiters import AsyncRateLimiter
from numpy.testing import assert_almost_equal
from settings import EnvSettings
from stable_baselines3 import PPO

from upkie.envs import UpkieGroundVelocity
from upkie.utils.filters import low_pass_filter
from upkie.utils.log_path import new_log_filename
from upkie.utils.raspi import configure_agent_process, on_raspi


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


async def run_policy(policy, logger: mpacklog.AsyncLogger):
    """
    Run policy, logging its actions and observations.

    Args:
        policy: Policy to run.
        logger: Logger to write actions and observations to.
    """
    rate = AsyncRateLimiter(EnvSettings().agent_frequency, "controller")
    assert_almost_equal(rate.dt, policy.env.get_attr("dt")[0])

    actions = np.zeros((1, 1))
    observations = policy.env.reset()
    floor_contact = False
    while True:
        await rate.sleep()

        actions, _ = (
            policy.predict(observations)
            if floor_contact
            else no_contact_policy(actions, rate.dt)
        )
        action_time = time.time()
        observations, _, dones, infos = policy.env.step(actions)
        floor_contact = infos[0]["observation"]["floor_contact"]["contact"]
        if dones[0]:
            observations = policy.env.reset()
            floor_contact = False

        await logger.put(
            {
                "action": infos[0]["action"],
                "observation": infos[0]["observation"],
                "policy": {
                    "action": actions[0],
                    "observation": observations[0],
                },
                "time": action_time,
            }
        )
    await logger.stop()


async def main(policy_path: str):
    env = UpkieGroundVelocity(shm_name="/vulp")
    policy = PPO("MlpPolicy", env, verbose=1)
    policy.set_parameters(policy_path)
    logger = mpacklog.AsyncLogger("/dev/shm/ppo_balancer.mpack")
    await asyncio.gather(run_policy(policy, logger), logger.write())
    policy.env.close()


if __name__ == "__main__":
    if on_raspi():
        configure_agent_process()

    agent_dir = os.path.abspath(os.path.dirname(__file__))
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--policy",
        help="path to the policy parameters file",
        default=f"{agent_dir}/policy/params.zip",
    )
    args = parser.parse_args()
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
