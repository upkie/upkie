#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2024 Inria

"""Try the wheeled inverted pendulum (WIP) environment.

This environment can be convenient for testing and debugging behaviors in a
perfect or noise-controlled setting.
"""

import gymnasium as gym
import numpy as np
import upkie.envs

upkie.envs.register()


if __name__ == "__main__":
    with gym.make(
        "WheeledInvertedPendulum-v1",
        frequency=200.0,
        frequency_checks=False,
        render_mode="plot",
    ) as env:
        observation, _ = env.reset()  # connects to the spine
        gain = np.array([10.0, 1.0, 0.0, 0.1])
        for step in range(1_000_000):
            # this is the same agent as in wheeled_balancing.py
            action = gain.dot(observation).reshape((1,))
            if step == 100:  # disturb the initial equilibrium
                action = env.action_space.sample()
            observation, reward, terminated, truncated, _ = env.step(action)
            if terminated or truncated:
                observation, _ = env.reset()
