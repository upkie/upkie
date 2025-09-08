#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2025 Inria
#
# /// script
# dependencies = ["pybullet>=3"]
# ///

"""Try basic balancing in a PyBullet simulation."""

import gymnasium as gym
import numpy as np

import upkie.envs

upkie.envs.register()

if __name__ == "__main__":
    with gym.make("Upkie-PyBullet-Pendulum", frequency=1000, gui=True) as env:
        observation, info = env.reset()
        while True:
            pitch = observation[0]
            ground_position = observation[1]
            ground_velocity = observation[3]
            v = 10.0 * pitch + 1.0 * ground_position + 0.1 * ground_velocity
            action = np.array([v])
            observation, reward, terminated, truncated, info = env.step(action)
            if terminated or truncated:
                observation, info = env.reset()
