#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2023 Inria

"""Balancing an upkie by PD feedback to wheel velocities."""

import gymnasium as gym

import upkie.envs

upkie.envs.register()

if __name__ == "__main__":
    with gym.make("Upkie-GroundVelocity-Spine", frequency=200.0) as env:
        observation, _ = env.reset()  # connects to the spine
        action = 0.0 * env.action_space.sample()
        for step in range(1_000_000):
            pitch = observation[0]
            ground_position = observation[1]
            ground_velocity = observation[3]
            v = 10.0 * pitch + 1.0 * ground_position + 0.1 * ground_velocity
            action[0] = v  # action is the next commanded ground velocity
            observation, reward, terminated, truncated, _ = env.step(action)
            env.unwrapped.log("pitch", pitch)
            if terminated or truncated:
                observation, _ = env.reset()
