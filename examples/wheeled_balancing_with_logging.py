#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2023 Inria

"""Wheel balancing with time-series logging of observations and actions."""

import asyncio
import time

import gym
import mpacklog
import numpy as np
from loop_rate_limiters import AsyncRateLimiter

import upkie.envs

upkie.envs.register()


async def balance(env: gym.Env, logger: mpacklog.Logger):
    """Run proportional balancer in gym environment with logging."""
    observation = env.reset()  # connects to the spine
    action = np.zeros(env.action_space.shape)
    rate = AsyncRateLimiter(frequency=200.0)
    for step in range(1_000_000):
        await rate.sleep()
        observation, reward, done, info = env.step(action)
        if done:
            observation = env.reset()
        pitch = observation[0]
        action[0] = 10.0 * pitch
        await logger.put(  # log info to be written to file later
            {
                "action": info["action"],
                "observation": info["observation"],
                "time": time.time(),
            }
        )


async def main():
    """Main function of our asyncio program."""
    logger = mpacklog.Logger("wheeled_balancing.mpack")
    with gym.make("UpkieWheelsEnv-v2") as env:
        await asyncio.gather(
            balance(env, logger),
            logger.write(),  # write logs to file when there is time
        )


if __name__ == "__main__":
    asyncio.run(main())
