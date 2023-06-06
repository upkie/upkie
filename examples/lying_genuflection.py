#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2023 Inria

"""Wheel proportional balancing with dictionary observations and actions."""

import gym
import numpy as np
from loop_rate_limiters import RateLimiter

import upkie.envs
from upkie.observers.base_pitch import compute_base_pitch_from_imu

nb_genuflections = 10
genuflection_steps = 200
amplitude = 2.0  # in radians

config = {
    "bullet": {
        "orientation_init_base_in_world": [0.707, 0.0, -0.707, 0.0],
        "position_init_base_in_world": [0.0, 0.0, 0.1],
    }
}

if __name__ == "__main__":
    upkie.envs.register()
    with gym.make("UpkieServosEnv-v1", config=config) as env:
        observation = env.reset()
        action = np.zeros(env.action_space.shape)
        rate = RateLimiter(frequency=200.0)
        for step in range(nb_genuflections * genuflection_steps):
            observation, reward, done, info = env.step(action)
            imu = info["observation"]["imu"]
            pitch = compute_base_pitch_from_imu(imu["orientation"])
            x = float(step % genuflection_steps) / genuflection_steps
            y = 2.0 * x * (1.0 - x)  # in [0, 1]
            A = amplitude  # in radians
            action[[0, 1, 3, 4]] = A * y * np.array([1.0, -2.0, -1.0, 2.0])
            rate.sleep()
