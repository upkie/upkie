#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2023 Inria

"""Wheel balancing with time-series logging of observations and actions."""

import asyncio
import os
import sys
import time

import gym
import mpacklog
import numpy as np

import upkie.envs

upkie.envs.register()

CPUID = 3  # CPU core to use on the Raspberry Pi


async def balance(env: gym.Env, logger: mpacklog.Logger):
    """Run proportional balancer in gym environment with logging."""
    observation = env.reset()  # connects to the spine
    action = np.zeros(env.action_space.shape)
    for step in range(1_000_000):
        observation, reward, done, info = await env.async_step(action)
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
    with gym.make("UpkieWheelsEnv-v2", frequency=200.0) as env:
        await asyncio.gather(balance(env, logger), logger.write())


if __name__ == "__main__":
    if os.geteuid() != 0:  # run as root so that we can set CPU affinity
        args = ["sudo", "-E", sys.executable] + sys.argv + [os.environ]
        os.execlpe("sudo", *args)
    os.sched_setaffinity(0, {CPUID})
    asyncio.run(main())
