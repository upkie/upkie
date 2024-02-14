#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2023 Inria

"""Balancing using proportional control from base pitch to wheel torques."""

import gymnasium as gym

import upkie.envs
from upkie.observers.base_pitch import compute_base_pitch_from_imu

upkie.envs.register()

GAIN = 10.0  # base pitch to wheel torque, in [N] * [m] / [rad]

if __name__ == "__main__":
    with gym.make("UpkieServos-v3", frequency=200.0) as env:
        action = env.get_neutral_action()

        # Disable position and velocity feedback in the wheels
        wheels = [servo for servo in action.keys() if "wheel" in servo]
        for wheel in wheels:
            action[wheel]["kp_scale"] = 0.0
            action[wheel]["kd_scale"] = 0.0

        obs, _ = env.reset()  # connects to the spine
        for step in range(1_000_000):
            pitch = compute_base_pitch_from_imu(obs["imu"]["orientation"])
            action["left_wheel"]["feedforward_torque"] = +GAIN * pitch
            action["right_wheel"]["feedforward_torque"] = -GAIN * pitch
            obs, _, terminated, truncated, _ = env.step(action)
            if terminated or truncated:
                obs, _ = env.reset()
