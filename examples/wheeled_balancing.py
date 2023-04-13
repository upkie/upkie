#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2023 Inria

import gym
import numpy as np
from loop_rate_limiters import RateLimiter

import upkie_locomotion.envs

upkie_locomotion.envs.register()


def balance(env: gym.Env):
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


if __name__ == "__main__":
    env = gym.make("UpkieWheelsEnv-v2")
    try:
        balance(env)
    finally:  # make sure we disconnect from the spine
        env.close()
