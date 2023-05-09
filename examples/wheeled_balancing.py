#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2023 Inria

"""Smallest example: balancing using a proportional wheel controller."""

import gym
import numpy as np
from loop_rate_limiters import RateLimiter

import upkie_locomotion.envs

if __name__ == "__main__":
    upkie_locomotion.envs.register()
    with gym.make("UpkieWheelsEnv-v2") as env:
        observation = env.reset()  # connects to the spine
        action = np.zeros(env.action_space.shape)
        rate = RateLimiter(frequency=200.0)
        for step in range(1_000_000):
            observation, reward, done, _ = env.step(action)
            if done:
                observation = env.reset()
            pitch = observation[0]
            action[0] = 10.0 * pitch
            rate.sleep()
