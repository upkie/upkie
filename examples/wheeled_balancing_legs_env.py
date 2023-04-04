#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2023 Inria

import gym
import numpy as np

from upkie_locomotion.observers.base_pitch import compute_base_pitch_from_imu
import upkie_locomotion.envs

upkie_locomotion.envs.register()


if __name__ == "__main__":
    env = gym.make("UpkieLegsEnv-v1")
    observation = env.reset(seed=42)

    action = 0.0 * env.action_space.sample()
    for step in range(1_000_000):
        observation, reward, done, info = env.step(action)
        imu = info["observation"]["imu"]
        if done:
            observation = env.reset()
        pitch = compute_base_pitch_from_imu(imu["orientation"])
        # action[1] = 1.0
        # action[3] = 1.0
        action[2] = np.nan  # no position target for left wheel
        action[5] = np.nan  # same for right wheel
        action[6 + 2] = 200.0 * pitch
        action[6 + 5] = -200.0 * pitch

    env.close()
