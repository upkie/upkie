#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2023 Inria

"""Wheel balancing using model predictive control of an LTV system.

This implementation is not efficient but more concise than the MPC balancer.
Check out the full agent to go further: https://github.com/upkie/mpc_balancer
"""

import gymnasium as gym
import numpy as np

import upkie.envs
from upkie.control.mpc_balancer import MPCBalancer

try:
    from qpmpc import solve_mpc
    from qpmpc.systems import WheeledInvertedPendulum
except ModuleNotFoundError as exn:
    raise MissingOptionalDependency(
        "This example uses qpmpc: `[conda|pip] install qpmpc`"
    ) from exn

upkie.envs.register()


if __name__ == "__main__":
    mpc_balancer = MPCBalancer()
    with gym.make("UpkieGroundVelocity", frequency=200.0) as env:
        _, info = env.reset()  # connects to the spine
        dt = env.unwrapped.dt
        commanded_velocity = 0.0
        target_ground_velocity = 0.0
        action = np.zeros(env.action_space.shape)
        for step in range(10_000):
            spine_observation = info["spine_observation"]
            action[0] = mpc_balancer.compute_ground_velocity(
                target_ground_velocity, spine_observation, dt
            )
            _, _, terminated, truncated, info = env.step(action)
            if terminated or truncated:
                _, info = env.reset()
                commanded_velocity = 0.0
