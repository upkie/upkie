#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2023 Inria

import gym
import numpy as np
from loop_rate_limiters import RateLimiter

import upkie_locomotion.envs
from upkie_locomotion.observers.base_pitch import compute_base_pitch_from_imu

upkie_locomotion.envs.register()


def balance(env: gym.Env):
    observation = env.reset()  # connects to the spine
    action = np.zeros(env.action_space.shape)
    rate = RateLimiter(frequency=200.0)
    for step in range(1_000_000):
        observation, reward, done, info = env.step(action)
        imu = info["observation"]["imu"]
        if done:
            env.reset()
        pitch = compute_base_pitch_from_imu(imu["orientation"])
        action[2] = np.nan  # no position target for left wheel
        action[5] = np.nan  # same for right wheel
        action[6 + 2] = 200.0 * pitch
        action[6 + 5] = -200.0 * pitch
        rate.sleep()


if __name__ == "__main__":
    env = gym.make("UpkieServosEnv-v1")
    try:
        balance(env)
    finally:  # make sure we disconnect from the spine
        env.close()
