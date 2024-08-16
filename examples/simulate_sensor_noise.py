#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2023 Inria

"""Balance in place despite sensor noise."""

import gymnasium as gym
import numpy as np

import upkie.envs

# The following values are completely arbitrary :)
JOINT_PROPS = {
    "torque_control_noise": 0.1,  # white noise in [N m]
    "torque_measurement_noise": 0.5,  # white noise in [N m]
}

if __name__ == "__main__":
    upkie.envs.register()
    with gym.make(
        "UpkieGroundVelocity-v3",
        frequency=200.0,
        spine_config={
            "bullet": {
                "imu_uncertainty": {
                    "accelerometer_noise": 0.2,
                    "gyroscope_noise": 0.05,
                },
                "joint_properties": {
                    "left_hip": JOINT_PROPS,
                    "left_knee": JOINT_PROPS,
                    "left_wheel": JOINT_PROPS,
                    "right_hip": JOINT_PROPS,
                    "right_knee": JOINT_PROPS,
                    "right_wheel": JOINT_PROPS,
                },
            }
        },
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
                observation, _ = env.reset()
