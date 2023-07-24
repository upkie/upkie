#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2022 Stéphane Caron

import argparse
import asyncio
import os
import time

import mpacklog
from loop_rate_limiters import AsyncRateLimiter
from settings import Settings
from stable_baselines3 import PPO
from upkie.envs import UpkieWheelsEnv


async def run_policy(policy, logger: mpacklog.Logger):
    """
    Run policy, logging its actions and observations.

    Args:
        policy: Policy to run.
        logger: Logger to write actions and observations to.
    """
    observation = policy.env.reset()
    agent_frequency = Settings().agent_frequency
    rate = AsyncRateLimiter(agent_frequency, "controller")
    for _ in range(1_000_000):
        await rate.sleep()
        action, _ = policy.predict(observation)
        action_time = time.time()
        observation, reward, done, info = policy.env.step(action)
        await logger.put(
            {
                "action": info[0]["action"],
                "observation": info[0]["observation"],
                "policy": {
                    "action": action[0],
                    "observation": observation[0],
                },
                "time": action_time,
            }
        )
    await logger.stop()


async def main(args: argparse.Namespace):
    agent_dir = os.path.abspath(os.path.dirname(__file__))
    env = UpkieWheelsEnv(shm_name="/vulp")
    policy = PPO("MlpPolicy", env, verbose=1)
    policy.set_parameters(f"{agent_dir}/policies/{args.name}")
    logger = mpacklog.Logger("/dev/shm/rollout.mpack")
    await asyncio.gather(run_policy(policy, logger), logger.write())
    policy.env.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("name", help="name of the policy to load")
    args = parser.parse_args()
    asyncio.run(main(args))
