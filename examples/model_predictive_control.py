#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2023 Inria

"""Wheel balancing using model predictive control of an LTV system."""

import gymnasium as gym
import numpy as np

import upkie.envs
from upkie.controllers import MPCBalancer

upkie.envs.register()

TARGET_GROUND_VELOCITY = 0.5  # m/s


if __name__ == "__main__":
    mpc_balancer = MPCBalancer()
    with gym.make("UpkieGroundVelocity", frequency=200.0) as env:
        _, info = env.reset()  # connects to the spine
        action = np.zeros(env.action_space.shape)
        for step in range(10_000):
            action[0] = mpc_balancer.compute_ground_velocity(
                target_ground_velocity=0.5,  # m/s
                spine_observation=info["spine_observation"],
                dt=env.unwrapped.dt,
            )
            _, _, terminated, truncated, info = env.step(action)
            if terminated or truncated:
                _, info = env.reset()
