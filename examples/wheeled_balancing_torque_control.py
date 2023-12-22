#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2023 Inria

"""Balancing using proportional control from base pitch to wheel torques."""

import gymnasium as gym
import numpy as np

import upkie.envs
from upkie.observers.base_pitch import compute_base_pitch_from_imu

upkie.envs.register()

if __name__ == "__main__":
    with gym.make("UpkieServos-v3", frequency=200.0) as env:
        action = env.get_default_action()

        # Stiff leg joints in zero configuration
        for leg_joint in ("left_hip", "left_knee", "right_hip", "right_knee"):
            action[leg_joint]["position"] = 0.0
            action[leg_joint]["velocity"] = 0.0

        # Disable position and velocity feedback in the wheels
        for wheel in ("left_wheel", "right_wheel"):
            action[wheel]["position"] = np.nan
            action[wheel]["velocity"] = 0.0
            action[wheel]["kp_scale"] = 0.0
            action[wheel]["kd_scale"] = 0.0

        obs, _ = env.reset()  # connects to the spine
        for step in range(1_000_000):
            pitch = compute_base_pitch_from_imu(obs["imu"]["orientation"])
            action["left_wheel"]["feedforward_torque"] = +10.0 * pitch
            action["right_wheel"]["feedforward_torque"] = -10.0 * pitch
            obs, _, terminated, truncated, _ = env.step(action)
            if terminated or truncated:
                obs, _ = env.reset()
