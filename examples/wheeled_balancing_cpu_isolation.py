#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2023 Inria

"""Wheel balancing with CPU isolation for real-time performance."""

import asyncio
import os
import sys
import time

import gym
import mpacklog
import numpy as np
import upkie_locomotion.envs
from loop_rate_limiters import AsyncRateLimiter

upkie_locomotion.envs.register()

CPUID = 3  # CPU core to use on the Raspberry Pi


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
        await logger.put(
            {
                "action": info["action"],
                "observation": info["observation"],
                "time": time.time(),
            }
        )


async def main():
    """Main function of our asyncio program."""
    env = gym.make("UpkieWheelsEnv-v2")
    logger = mpacklog.Logger("wheeled_balancing.mpack")
    try:
        await asyncio.gather(balance(env, logger), logger.write())
    finally:
        env.close()


if __name__ == "__main__":
    if os.geteuid() != 0:  # run as root so that we can set CPU affinity
        args = ["sudo", "-E", sys.executable] + sys.argv + [os.environ]
        os.execlpe("sudo", *args)
    os.sched_setaffinity(0, {CPUID})
    asyncio.run(main())
