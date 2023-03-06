#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2022 Stéphane Caron

import argparse
import asyncio
import os
import time

import gin
import mpacklog
from loop_rate_limiters import AsyncRateLimiter
from settings import Settings
from stable_baselines3 import PPO

from upkie_locomotion.envs import UpkieWheelsEnv

keep_going = True


async def _run_policy(policy, logger: mpacklog.Logger) -> None:
    observation = policy.env.reset()
    brain_frequency = Settings().brain_frequency
    rate = AsyncRateLimiter(brain_frequency, "controller")
    upkie_env = policy.env.envs[0].env
    while keep_going:
        await rate.sleep()
        action, _ = policy.predict(observation)
        action_time = time.time()
        observation, reward, done, _ = policy.env.step(action)
        await logger.put(
            {
                "action": upkie_env.last_action,
                "observation": upkie_env.last_observation,
                "policy": {
                    "action": action[0],
                    "observation": observation[0],
                },
                "time": action_time,
            }
        )
    await logger.stop()


async def run_policy(policy):
    logger = mpacklog.Logger("/dev/shm/rollout.mpack")
    await asyncio.gather(
        _run_policy(policy, logger),
        logger.write(),
    )


def load_policy(agent_dir: str, policy_name: str):
    env = UpkieWheelsEnv(shm_name="/vulp")
    policy = PPO("MlpPolicy", env, verbose=1)
    policy.set_parameters(f"{agent_dir}/policies/{policy_name}")
    return policy


if __name__ == "__main__":
    agent_dir = os.path.abspath(os.path.dirname(__file__))
    gin.parse_config_file(f"{agent_dir}/settings.gin")

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("name", help="name of the policy to load")
    args = parser.parse_args()

    policy = load_policy(agent_dir, args.name)
    asyncio.run(run_policy(policy))
    policy.env.close()
