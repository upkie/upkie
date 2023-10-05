#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2023 Inria

"""Genuflect while lying on a horizontal floor."""

import gymnasium as gym
import numpy as np

import upkie.envs

nb_genuflections = 10
genuflection_steps = 200
amplitude = 1.0  # in radians

spine_config = {
    "bullet": {
        "reset": {
            "orientation_base_in_world": [0.707, 0.0, -0.707, 0.0],
            "position_base_in_world": [0.0, 0.0, 0.1],
        },
    }
}

if __name__ == "__main__":
    upkie.envs.register()
    with gym.make(
        "UpkieServos-v2", spine_config=spine_config, frequency=200.0
    ) as env:
        env.reset()  # connects to the spine
        action = np.zeros(env.action_space.shape)
        for step in range(nb_genuflections * genuflection_steps):
            observation, _, _, _, _ = env.step(action)
            x = float(step % genuflection_steps) / genuflection_steps
            y = 4.0 * x * (1.0 - x)  # in [0, 1]
            A = amplitude  # in radians

            left_hip = 0  # index of left hip position in vectorized action
            left_knee = 1  # same for left knee, etc.
            right_hip = 3
            right_knee = 4
            q_0134 = A * y * np.array([1.0, -2.0, -1.0, 2.0])
            action[[left_hip, left_knee, right_hip, right_knee]] = q_0134
