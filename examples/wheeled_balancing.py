#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2023 Inria

"""Tiny example: balancing using a proportional wheel controller."""

import gymnasium as gym
import upkie.envs

upkie.envs.register()

if __name__ == "__main__":
    with gym.make("UpkieGroundVelocity-v3", frequency=200.0) as env:
        observation, _ = env.reset()  # connects to the spine
        action = 0.0 * env.action_space.sample()
        for step in range(1_000_000):
            pitch = observation[0]
            action[0] = 10.0 * pitch  # 1D action: [ground_velocity]
            observation, reward, terminated, truncated, _ = env.step(action)
            if terminated or truncated:
                observation, _ = env.reset()
