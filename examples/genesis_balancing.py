#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0

"""Try Genesis environment with basic balancing."""

import gymnasium as gym
import numpy as np

import upkie.envs

upkie.envs.register()

if __name__ == "__main__":
    with gym.make(
        "Upkie-Genesis-Pendulum",
        frequency=200,
        frequency_checks=False,
        genesis_init={
            "debug": False,
            "logging_level": None,
            "performance_mode": False,
            "precision": "32",
            "seed": 42,
        },
        gui=True,
    ) as env:
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
