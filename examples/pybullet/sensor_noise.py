#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2023 Inria
#
# /// script
# dependencies = ["upkie", "pybullet>=3"]
# ///

"""Balance in place despite sensor noise."""

import gymnasium as gym
import numpy as np

import upkie.envs

# The following values are completely arbitrary :)
JOINT_NOISE = {
    # Try increasing torque control noise: it affects balancing
    "torque_control_noise": 0.5,  # white noise in N⋅m
    "torque_measurement_noise": 0.1,  # white noise in N⋅m
}

if __name__ == "__main__":
    upkie.envs.register()
    with gym.make(
        "Upkie-PyBullet-Pendulum",
        frequency=200.0,
        bullet_config={
            "joint_properties": {
                "left_hip": JOINT_NOISE,
                "left_knee": JOINT_NOISE,
                "left_wheel": JOINT_NOISE,
                "right_hip": JOINT_NOISE,
                "right_knee": JOINT_NOISE,
                "right_wheel": JOINT_NOISE,
            }
        },
    ) as env:
        observation, _ = env.reset()  # connects to the spine
        gain = np.array([10.0, 1.0, 0.1, 0.1])
        for step in range(1_000_000):
            # this is the same agent as in wheeled_balancing.py
            action = gain.dot(observation).reshape((1,))
            if step == 100:  # disturb the initial equilibrium
                action = env.action_space.sample()
            observation, reward, terminated, truncated, info = env.step(action)
            left_knee_torque = info["spine_observation"]["servo"]["left_knee"][
                "torque"
            ]
            print(f"{left_knee_torque=}")
            if terminated or truncated:
                observation, _ = env.reset()
