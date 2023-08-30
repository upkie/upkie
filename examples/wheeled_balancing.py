#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2023 Inria

"""Tiny example: balancing using a proportional wheel controller."""

import gymnasium as gym
import numpy as np

import upkie.envs

upkie.envs.register()

if __name__ == "__main__":
    with gym.make("UpkieGroundVelocity-v1", frequency=200.0) as env:
        env.reset()  # connects to the spine
        action = np.zeros(env.action_space.shape)
        for step in range(1_000_000):
            observation, reward, terminated, truncated, _ = env.step(action)
            if terminated or truncated:
                observation, _ = env.reset()
            pitch = observation[0]
            action[0] = 10.0 * pitch  # 1D action: [ground_velocity]
